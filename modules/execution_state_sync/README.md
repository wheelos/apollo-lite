# Execution state synchronization

Local, transactional lifecycle transport for Mission, Planning and Control.
Serialized protobuf payloads are opaque: this library does not depend on mission
messages, decide success, perform planning, or control actuators.

## Contract

Use one persistent **local** database file per vehicle, mapped to the same file
in every participant. Network filesystems and in-memory databases are unsupported.
`Store::Open` enables WAL, FULL synchronous commits, bounded busy waiting and a
versioned schema. Opening a role claims its single writer slot with a fresh
epoch and monotonically increasing fencing generation. A live claim cannot be
duplicated; claims use host boot identity plus the shared monotonic clock and a
bounded lease. Every write is recorded with the Store-assigned generation.
Worker snapshots periodically renew the lease; synchronous Store calls renew it
on use. Lease expiry makes a participant appear not ready but does not by
itself change its generation. A live old connection can renew only while no
newer generation has taken the role; after takeover its operations are fenced.
`ReleaseWriter()` supports explicit orderly release, and Store destruction also
attempts to release its own claim.

Each role registers its contract version and implemented capability names.
`Client::Ready()` becomes true only after it has acknowledged the startup tail
and committed readiness for the current epoch. `Participants()` reports the
current live readiness, capability set, contract version and fencing generation;
expired or prior-boot sessions are not ready. This readiness is separate from
the persisted event history and does not restore old execution authority.

Each connection is thread-confined; separate roles may operate concurrently.
All methods return `Result`; output arguments remain unchanged on failure.
Inspect both `code` and `message` (and `sqlite_code` for storage failures). No
implicit retry occurs.

| Writer role | Channels |
| --- | --- |
| `kMission` | `kMission` |
| `kMission` | `kSafetyRequest` |
| `kPlanning` | `kMotion`, `kPlanningStatus` |
| `kControl` | `kControlStatus`, `kSafetyStatus` |

### Lifecycle status ownership

The execution-state database is the canonical inter-component lifecycle
channel. Components consume its ordered, committed events; Cyber topics are
compatibility/observation mirrors published only after the corresponding
durable commit succeeds. A topic mirror must not be treated as a second source
of execution authority.

| Owner | Reports | Does not decide |
| --- | --- | --- |
| Mission | Request disposition and public task lifecycle/result | Planning admission or Control execution facts |
| Planning | Directive admission and its Mission-session/execution view | Control executor facts or Mission request ownership |
| Control | Motion-executor outcome and Control/safety observations | Mission cancellation or public task completion |

Planning associates each Control motion status with the current Mission and
Motion identities before changing its session view. Planning then reports that
view through `kPlanningStatus`; Mission alone maps a correlated Planning
terminal report to the typed request ledger and public Mission status.
`kControlStatus` distinguishes an unguarded owner runtime observation from a
Motion result. Results require both Mission and Motion parents; historical
parent events remain valid references after a newer head commits. Runtime
observations carry no causal guards and cannot authorize task transitions.
`MOTION_EXECUTION_CANCELLED` describes a Control motion operation, not a
Mission cancellation: only a Mission cancel directive starts that lifecycle
transition, and Planning confirms it after the required stop/hold evidence.
Likewise, Control safety state is an observation, not a Mission terminal
disposition.

Safety request/status events are independent of mission-motion guards so an
authorized Mission safety request can be enforced and observed without an
active motion command. They do not set or clear the Mission cancellation fence.
The typed controlled-stop request latches Control's safety policy without
cancelling the Mission; reset requires the exact Control safety identity and
fresh stationary evidence. Control restores a still-latched stop from the latest
durable safety observation after restart and publishes a new observation with
its current Control epoch. Mission correlates durable Control observations to
the originating request ledger before reporting an outcome.

An operation carries an immutable operation ID, identity
`{epoch, aggregate_id, command_id, revision}`, protobuf bytes and an explicit
`expected_revision`. Revisions are **channel-wide**, including across aggregate
or process epoch changes: the next revision must equal expected + 1. Sequence
numbers order all channels globally. The event append and latest projection
update commit in one transaction. Exact ID retries return the original commit,
even if subsequent revisions exist; any changed field conflicts. Guard order
does not affect idempotency.

Every lifecycle descendant requires a `kMission` guard naming its Mission
**event sequence**, not its revision. Control Motion results additionally
require a `kMotion` sequence. Planning runtime status may include an optional
Motion guard. Control result observations may reference historical immutable
parents; their consumers validate identity correlation before applying outcomes.
Guards are checked atomically with CAS and commit. Mission can set
`mission_fenced`; motion then fails with `kFenced`, except explicit
`allow_when_fenced` stop/cleanup operations. Only mission writes the fence and
only motion may request its exemption. Callers must validate that exempt payloads
really are safe stop/cleanup commands: the store cannot inspect opaque protobufs.
Mission and motion guards do not themselves prove their payload identities match;
callers validate causal identity, terminal-state transitions and authorization.

`ReadSnapshot` returns six latest projections at one transactional high-watermark.
It is for coherent observation, **not** ordered command delivery. Consume
`ReadEvents(after, limit)` in global order, including records irrelevant to your
role when advancing the cursor. Batches are limited to 64 events and each payload
to 1 MiB. `next_sequence` is the last returned sequence; `high_watermark` may be
later. Poll again to drain subsequent batches. History is append-only on disk;
there is intentionally no silent pruning or unbounded replay allocation.
Provision and monitor disk capacity; retention/migration needs an explicit
cross-consumer policy.

`Consumer{role, epoch}` keys a persistent acknowledgement cursor. Missing means
zero. Acknowledge only after processing the entire ordered prefix, using cursor
CAS. Only the owning role may acknowledge. Reusing the same consumer epoch after
restart resumes replay; choosing a new epoch starts at zero. Operation identity
epochs and consumer replay epochs have different purposes. The store cannot
prove processing happened, so arbitrary forward acknowledgements are caller
errors. Processing and physical effects are **at least once**, not exactly once;
consumers must deduplicate and reconcile effects after a crash.

## Minimal use

```cpp
using namespace apollo::execution_state_sync;
std::unique_ptr<Store> store;
Result result = Store::Open({"/vehicle/state/execution.db", Role::kMission},
                            &store);
if (!result.ok()) { /* surface fault; do not claim acceptance */ }
Operation op;
op.operation_id = "durable-request-id";
op.identity = {"authorization-epoch", "mission-id", "command-id", 1};
op.expected_revision = 0;
op.payload = serialized_mission_protobuf;
Commit commit;
result = store->Submit(op, &commit);
```

Parent directories must already exist. Commit success means persisted intent,
not acceptance, execution, stopping or business completion. Retain the exact
operation ID and content across uncertain-result retries.

The shared gflag is declared by `:execution_state_gflags`:

```text
--execution_state_db_path=/apollo/data/execution-state-<vehicle-id>.db
```

Its compiled default is deliberately empty and `Client::Init` fails unless the
value is an absolute path whose parent directory already exists. `/apollo/data`
exists in the standard deployment, but deployment must choose a per-vehicle
filename and pass the identical value to Mission, Planning and Control. The
library does not create directories.

## Asynchronous adapter

Link `:worker` and create `Worker::Start` on a lifecycle thread. The worker owns
its connection. `TrySubmit`, `TryReadEvents`, `TryReadCursor` and
`TryAcknowledge` return an admission result and ticket; the eventual storage
result arrives via `TryTakeCompletion`. Admission is **not** persistence.
All accepted requests retain a capacity slot until their completion is consumed,
including failures. Queue contention returns `kBusy`, saturation `kQueueFull`.
Requests execute FIFO. Event completions remain bounded by capacity × 64 × 1 MiB
plus metadata; choose small capacity/batch sizes for the deployment.

`Latest()` publishes immutable `WorkerView{result,snapshot,observed_at}` without
SQLite access on the caller thread. Failed reads retain the last good snapshot
and its original observation time while exposing the failure. Never interpret
that retained snapshot as renewed authority. Check result and snapshot age.
Snapshot publications can skip intermediate events: use event requests and
explicit acknowledgements for lifecycle processing.

No callbacks execute under queue/storage locks. Try methods use queue try-locks;
atomic shared-pointer publication is not guaranteed lock-free by C++17.
This is not a hard-real-time interface. Allocations, payload copies and shared
pointer destruction also have costs. `Stop`/destruction drains accepted requests
and joins, so invoke it outside control callbacks and not concurrently with other
methods. Caller-retained views/completions consume caller-owned memory.

## Component client

`Client` wraps `Worker` for component integration:

```cpp
Client client;
Result result = client.Init(FLAGS_execution_state_db_path, Role::kPlanning,
                            fresh_process_epoch);
result = client.Submit(Channel::kMotion, serialized_directive,
                       {{Channel::kMission, mission_event_sequence}},
                       false, retirement_cleanup, &ticket);
result = client.Poll(&ordered_events);
result = client.Acknowledge(ordered_events.back().sequence);
while (client.TakeSubmission(&submission).ok()) {
  // submission.result is the commit result, distinct from Submit admission.
}
```

The client assigns store revisions per channel; protobuf identity revisions
remain domain data and are not reused as transport CAS revisions. Submissions
are bounded and serialized per channel. A duplicate payload/guard submission
while pending returns the existing client ticket. Different work is queued only
within configured capacity. Every commit result must be consumed.

A `kControlStatus` result must reference committed Mission and Motion parents;
newer channel heads do not relabel or invalidate those immutable references.
Parent conflicts are returned through `TakeSubmission` but do not latch Client
health. Conflicts on Mission or Motion submissions remain latched safety
faults. All storage, corruption, fencing and internal failures remain fatal
for every channel.

Initialization uses a fresh durable consumer epoch and asynchronously
acknowledges the database tail before `Ready()` becomes true. It deliberately
does not replay persisted operations as live authorization. Components must
remain fail-safe until ready and require a new live Mission event after restart.
After readiness, `Poll` delivers bounded globally ordered batches. Call
`Acknowledge` only after processing the entire batch; acknowledgement admission
is retained and retried internally under transient worker backpressure.

## Caller safety obligations and validation

Persisted RUNNING records never automatically restore actuator authority after
restart. Domain layers enforce fresh epochs, terminal-state immutability,
deadlines, observation time, causal identities, cancellation stop/hold handoff,
and payload schema/capability validation. API roles are ownership checks, not a
security boundary against direct database access. Use OS filesystem permissions.
Database linearization does not imply instantaneous actuator revocation.
Control must use local expiry/watchdog and safe fallback when storage, worker
admission, completion, authorization freshness or communication fails.

Managed-container unit target:
`bazel test //modules/execution_state_sync:store_test`.
Tests cover owner policy, parent CAS/fences, immutable IDs, binary payloads,
writer claims/readiness/capabilities, expired-lease takeover and stale-writer
fencing, restart/replay/cursors, bounded batches, busy errors, transaction
rollback, corruption, preserved outputs and worker backpressure.

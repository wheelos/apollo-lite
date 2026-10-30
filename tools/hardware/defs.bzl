"""Hardware selection and resolved product manifest."""

load("@bazel_skylib//rules:common_settings.bzl", "BuildSettingInfo")
load("@hardware.build//tools/hardware:defs.bzl", _VEHICLES = "VEHICLES")
load(":runtime_paths.bzl", "deployment_path")

HardwareSelectionInfo = provider(fields = ["choices"])
VEHICLES = _VEHICLES

LIDARS = {
    "hesai": ["@hardware.lidar//hesai:libhesai_component.so"],
    "livox": ["@hardware.lidar//livox:liblivox_component.so"],
    "lslidar": ["@hardware.lidar//lslidar:liblslidar_component.so"],
    "rslidar": ["@hardware.lidar//rslidar:librslidar_component.so"],
    "seyond": ["@hardware.lidar//seyond:libseyond_component.so"],
    "vanjeelidar": ["@hardware.lidar//vanjeelidar:libvanjeelidar_component.so"],
    "velodyne": [
        "@hardware.lidar//velodyne/compensator:libvelodyne_compensator_component.so",
        "@hardware.lidar//velodyne/driver:libvelodyne_driver_component.so",
        "@hardware.lidar//velodyne/fusion:libvelodyne_fusion_component.so",
        "@hardware.lidar//velodyne/parser:libvelodyne_convert_component.so",
    ],
}

def enabled(name, targets):
    return select({
        "@hardware.build//tools/hardware:" + name: targets,
        "//conditions:default": [],
    })

def lidar_targets(runtime = False):
    result = []
    for brand, targets in LIDARS.items():
        result += enabled(
            "with_lidar_" + brand,
            ["@hardware.lidar//" + brand + ":runtime_data"] if runtime else targets,
        )
    return result

def _manifest_impl(ctx):
    choices = {
        key: target[BuildSettingInfo].value
        for target, key in ctx.attr.settings.items()
    }
    legacy_vehicle = ctx.var.get("vehicle", "")
    vehicle = choices["vehicle"]
    if vehicle == "legacy":
        vehicle = legacy_vehicle or "none"
    elif legacy_vehicle and legacy_vehicle != vehicle:
        fail("Conflicting vehicle selections: --define=vehicle={} and --@hardware.build//tools/hardware:vehicle={}".format(legacy_vehicle, vehicle))
    if vehicle not in VEHICLES + ["none"]:
        fail("Unknown vehicle: " + vehicle)
    if vehicle != "none" and choices["canbus"] == "none":
        fail("A selected vehicle requires CAN transport")
    choices["vehicle"] = vehicle
    gnss_config = []
    if choices["gnss"] != "none":
        gnss_format = {
            "all": "NOVATEL_BINARY",
            "novatel": "NOVATEL_BINARY",
            "huace": "HUACE_TEXT",
            "forsense": "FORSENSE_BINARY",
        }[choices["gnss"]]
        config = ctx.actions.declare_file("gnss_conf.pb.txt")
        ctx.actions.write(config, """# Example only: verify port, receiver configuration and calibration before use.
data {
  format: %s
  serial { device: "/dev/ttyACM0" baud_rate: 115200 }
}
rtk_solution_type: RTK_RECEIVER_SOLUTION
""" % gnss_format)
        gnss_config.append(config)
    vehicle_config = []
    if vehicle != "none":
        brand = {"devkit": "DKIT", "lincoln": "LINCOLN_MKZ", "neolix_edu": "NEOLIX"}.get(vehicle, vehicle.upper())
        can_brand = "FAKE_CAN" if choices["canbus"] == "fake" else "SOCKET_CAN_RAW"
        config = ctx.actions.declare_file("canbus_conf.pb.txt")
        ctx.actions.write(config, """# Example only: verify vehicle parameters and CAN interface before use.
vehicle_parameter {
  brand: %s
  max_enable_fail_attempt: 5
  driving_mode: COMPLETE_MANUAL
}
can_card_parameter {
  brand: %s
  channel_id: CHANNEL_ID_ZERO
  num_ports: 8
  interface: NATIVE
}
enable_debug_mode: false
enable_receiver_log: false
enable_sender_log: false
""" % (brand, can_brand))
        vehicle_config.append(config)
    result = {
        "schema_version": 1,
        "profile": ctx.attr.profile[BuildSettingInfo].value,
        "choices": choices,
        "components": [str(target.label) for target in ctx.attr.components],
        "component_files": [
            "bazel-bin/" + deployment_path(f)
            for f in ctx.files.components
        ],
        "runtime_files": sorted([f.short_path for f in ctx.files.runtime_data]),
        "generated_examples": [f.basename for f in gnss_config + vehicle_config],
        "scope": "matched hardware and mainboard runtime; target-system libraries and deployment resources required",
    }
    output = ctx.actions.declare_file(ctx.label.name + ".json")
    ctx.actions.write(output, json.encode_indent(result, indent = "  ") + "\n")
    return [
        HardwareSelectionInfo(choices = choices),
        DefaultInfo(files = depset([output, ctx.info_file])),
        OutputGroupInfo(
            gnss_config = depset(gnss_config),
            vehicle_config = depset(vehicle_config),
        ),
    ]

hardware_manifest = rule(
    implementation = _manifest_impl,
    attrs = {
        "components": attr.label_list(allow_files = True),
        "profile": attr.label(mandatory = True),
        "runtime_data": attr.label_list(allow_files = True),
        "settings": attr.label_keyed_string_dict(),
    },
)

const state = {
  plugins: [],
  selectedPlugin: null,
  jobs: [],
  selectedJobId: null,
  pollHandle: null,
  jobDetailScroll: {
    summary: null,
    logs: null,
  },
};

async function fetchJson(url, options) {
  const response = await fetch(url, options);
  if (!response.ok) {
    throw new Error(`${response.status} ${response.statusText}`);
  }
  return response.json();
}

function statusClass(ok) {
  return ok ? "status-ok" : "status-bad";
}

function isPluginAvailable(plugin) {
  return Boolean(plugin && plugin.probe && plugin.probe.available);
}

function parseExpectedValue(raw) {
  if (raw === "true") return true;
  if (raw === "false") return false;
  if (raw === "" || raw == null) return "";
  if (!Number.isNaN(Number(raw))) return Number(raw);
  return raw;
}

function getFieldInput(root, actionId, fieldName) {
  return root.querySelector(`#field-${actionId}-${fieldName}`);
}

function getFieldValue(root, actionId, field) {
  const input = getFieldInput(root, actionId, field.name);
  if (!input) return undefined;
  if (field.type === "checkbox") return input.checked;
  if (field.type === "number") return Number(input.value || 0);
  return input.value.trim();
}

function renderPlugins() {
  const container = document.getElementById("plugin-list");
  container.innerHTML = "";
  state.plugins.forEach((plugin) => {
    const card = document.createElement("div");
    const selectedPluginId = state.selectedPlugin ? state.selectedPlugin.plugin_id : "";
    const available = isPluginAvailable(plugin);
    card.className =
      "plugin-card" +
      (selectedPluginId === plugin.plugin_id ? " selected" : "") +
      (available ? "" : " disabled");
    const probe = plugin.probe || {};
    card.innerHTML = `
      <div class="panel-title-row">
        <strong>${plugin.name}</strong>
        <span class="status-pill ${statusClass(Boolean(probe.available))}">
          ${probe.available ? "Available" : "Unavailable"}
        </span>
      </div>
      <div class="subtle">${plugin.description}</div>
    `;
    if (available) {
      card.onclick = () => {
        state.selectedPlugin = plugin;
        renderPlugins();
        renderPluginDetail();
      };
    }
    container.appendChild(card);
  });
}

function createField(field, actionId) {
  const wrapper = document.createElement("div");
  wrapper.className = "field" + (field.type === "path" || field.type === "text" ? " field-wide" : "");
  wrapper.dataset.fieldName = field.name;
  if (field.visible_when) {
    wrapper.dataset.visibleWhenField = field.visible_when.field;
    wrapper.dataset.visibleWhenEquals = String(field.visible_when.equals);
  }
  if (field.type === "checkbox") {
    wrapper.classList.add("field-checkbox", "field-wide");
    wrapper.innerHTML = `
      <label class="checkbox-row">
        <input id="field-${actionId}-${field.name}" type="checkbox" ${field.default ? "checked" : ""}>
        <span>${field.label}${field.required ? " *" : ""}</span>
      </label>
    `;
    return wrapper;
  }
  wrapper.innerHTML = `
    <label>${field.label}${field.required ? " *" : ""}</label>
    <input id="field-${actionId}-${field.name}" type="${field.type === "number" ? "number" : "text"}"
      value="${field.default ?? ""}" placeholder="${field.placeholder ?? ""}">
  `;
  return wrapper;
}

function applyFieldVisibility(root, action) {
  (action.fields || []).forEach((field) => {
    const wrapper = root.querySelector(`.field[data-field-name="${field.name}"]`);
    if (!wrapper || !field.visible_when) return;
    const expected = parseExpectedValue(String(field.visible_when.equals));
    const controller = (action.fields || []).find((item) => item.name === field.visible_when.field);
    const currentValue = controller ? getFieldValue(root, action.action_id, controller) : undefined;
    const visible = currentValue === expected;
    wrapper.classList.toggle("hidden", !visible);
    const input = getFieldInput(root, action.action_id, field.name);
    if (input) {
      input.disabled = !visible;
    }
  });
}

function buildActionForm(plugin, action) {
  const wrapper = document.createElement("div");
  wrapper.className = "action-card";
  const liveUrl = action.live_url;
  wrapper.innerHTML = `<h3>${action.title}</h3>`;
  if (action.description) {
    const text = document.createElement("p");
    text.className = "subtle action-description";
    text.textContent = action.description;
    wrapper.appendChild(text);
  }
  if (liveUrl) {
    const button = document.createElement("button");
    button.textContent = "Open Live Viewer";
    button.onclick = () => window.open(liveUrl, "_blank");
    wrapper.appendChild(button);
    return wrapper;
  }

  const grid = document.createElement("div");
  grid.className = "field-grid";
  (action.fields || []).forEach((field) => {
    grid.appendChild(createField(field, action.action_id));
  });
  wrapper.appendChild(grid);
  (action.fields || []).forEach((field) => {
    const input = getFieldInput(wrapper, action.action_id, field.name);
    if (!input) return;
    input.addEventListener("change", () => applyFieldVisibility(wrapper, action));
    input.addEventListener("input", () => applyFieldVisibility(wrapper, action));
  });
  applyFieldVisibility(wrapper, action);

  const actionsBar = document.createElement("div");
  actionsBar.className = "actions-bar";
  const hasRecordField = (action.fields || []).some((field) => field.name === "data_package");
  if (hasRecordField) {
    const inspectButton = document.createElement("button");
    inspectButton.textContent = "Inspect Record";
    inspectButton.onclick = async () => {
      const recordField = getFieldInput(wrapper, action.action_id, "data_package");
      if (!recordField || !recordField.value) return;
      try {
        const data = await fetchJson(
          `/api/plugins/${plugin.plugin_id}/inspect_record?action_id=${action.action_id}&path=${encodeURIComponent(recordField.value)}`
        );
        alert(
          [
            `PointCloud Topics:\n${(data.pointcloud_topics || []).join("\n") || "(none)"}`,
            `\nLocalization Topics:\n${(data.localization_topics || []).join("\n") || "(none)"}`,
          ].join("\n")
        );
      } catch (error) {
        alert(`Inspect failed: ${error}`);
      }
    };
    actionsBar.appendChild(inspectButton);
  }
  const runButton = document.createElement("button");
  runButton.textContent = "Run";
  runButton.onclick = async () => {
    const params = {};
    (action.fields || []).forEach((field) => {
      const input = getFieldInput(wrapper, action.action_id, field.name);
      if (!input || input.disabled) return;
      if (field.type === "checkbox") {
        params[field.name] = input.checked;
        return;
      }
      params[field.name] = field.type === "number" ? Number(input.value || 0) : input.value.trim();
    });
    const job = await fetchJson("/api/jobs", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({
        plugin_id: plugin.plugin_id,
        action_id: action.action_id,
        params,
      }),
    });
    state.selectedJobId = job.job_id;
    await loadJobs();
    renderJobDetail();
  };
  actionsBar.appendChild(runButton);
  wrapper.appendChild(actionsBar);
  return wrapper;
}

function renderPluginDetail() {
  const detail = document.getElementById("plugin-detail");
  if (!state.selectedPlugin) {
    detail.innerHTML = `
      <div class="compact-empty-state">
        <p class="eyebrow">Tool</p>
        <h2>Select A Tool</h2>
        <p class="subtle">After selecting a tool on the left, the launch parameters and execution entry point are displayed here directly.</p>
      </div>
    `;
    return;
  }
  detail.innerHTML = `
    <div class="plugin-header">
      <div class="panel-title-row panel-title-row-tight">
        <p class="eyebrow">Plugin</p>
        <span class="status-pill ${statusClass(Boolean(state.selectedPlugin.probe && state.selectedPlugin.probe.available))}">
          ${state.selectedPlugin.probe && state.selectedPlugin.probe.available ? "Available" : "Unavailable"}
        </span>
      </div>
      <div>
        <h2>${state.selectedPlugin.name}</h2>
        <p class="subtle">${state.selectedPlugin.description}</p>
      </div>
    </div>
  `;
  if (!isPluginAvailable(state.selectedPlugin)) {
    const probe = state.selectedPlugin.probe || {};
    const missing = probe.missing || [];
    const warnings = probe.warnings || [];
    const unavailableBox = document.createElement("div");
    unavailableBox.className = "warning-box";
    unavailableBox.innerHTML = `
      <p>This plugin is unavailable in the current repository and cannot be launched.</p>
      ${missing.length ? `<p>Missing: ${missing.join(", ")}</p>` : ""}
      ${warnings.length ? `<p>Notes: ${warnings.join(" ")}</p>` : ""}
    `;
    detail.appendChild(unavailableBox);
    return;
  }
  const warnings = (state.selectedPlugin.probe && state.selectedPlugin.probe.warnings) || [];
  if (warnings.length) {
    const warningBox = document.createElement("div");
    warningBox.className = "warning-box";
    warningBox.innerHTML = warnings.map((item) => `<p>${item}</p>`).join("");
    detail.appendChild(warningBox);
  }
  (state.selectedPlugin.actions || []).forEach((action) => {
    detail.appendChild(buildActionForm(state.selectedPlugin, action));
  });
}

function renderJobs() {
  const container = document.getElementById("job-list");
  container.innerHTML = "";
  state.jobs
    .slice()
    .reverse()
    .forEach((job) => {
      const card = document.createElement("div");
      card.className = "job-card" + (state.selectedJobId === job.job_id ? " selected" : "");
      card.innerHTML = `
        <div class="panel-title-row">
          <strong>${job.plugin_id} / ${job.action_id}</strong>
          <span class="status-pill ${job.status === "completed" ? "status-ok" : job.status === "failed" ? "status-bad" : "status-warn"}">
            ${job.status}
          </span>
        </div>
        <div class="subtle">${job.job_id}</div>
        <div class="subtle">${job.stage} · ${job.progress.toFixed(1)}%</div>
      `;
      card.onclick = () => {
        state.selectedJobId = job.job_id;
        renderJobs();
        renderJobDetail();
      };
      container.appendChild(card);
    });
}

function renderJobDetail() {
  const panel = document.getElementById("job-panel");
  const detail = document.getElementById("job-detail");
  const previousSummaryBox = detail.querySelector(".summary-box");
  const previousLogBox = detail.querySelector(".log-box");
  if (previousSummaryBox) {
    state.jobDetailScroll.summary = {
      top: previousSummaryBox.scrollTop,
      atBottom:
        previousSummaryBox.scrollHeight - previousSummaryBox.clientHeight - previousSummaryBox.scrollTop < 24,
    };
  }
  if (previousLogBox) {
    state.jobDetailScroll.logs = {
      top: previousLogBox.scrollTop,
      atBottom:
        previousLogBox.scrollHeight - previousLogBox.clientHeight - previousLogBox.scrollTop < 24,
    };
  }
  if (!state.selectedJobId) {
    panel.classList.add("hidden");
    return;
  }
  const job = state.jobs.find((item) => item.job_id === state.selectedJobId);
  if (!job) {
    panel.classList.add("hidden");
    return;
  }
  panel.classList.remove("hidden");
  const artifacts = (job.artifacts || [])
    .map((artifact) => {
      if (artifact.default_entry) {
        const href = `/artifacts/${job.job_id}/${artifact.key}/${artifact.default_entry}`;
        return `<li><a href="${href}" target="_blank">${artifact.label}</a> <span class="subtle">${artifact.root}</span></li>`;
      }
      return `<li><strong>${artifact.label}</strong> <span class="subtle">${artifact.root}</span></li>`;
    })
    .join("");
  const viewerUrl = job.summary ? job.summary.viewer_url : "";
  const resultDir = job.summary ? job.summary.result_dir : "";
  detail.innerHTML = `
    <div class="subtle">${job.job_id}</div>
    <div class="progress"><span style="width:${job.progress}%;"></span></div>
    <p><strong>${job.stage}</strong> · ${job.message || "-"}</p>
    ${job.error ? `<p class="status-bad"><strong>Error:</strong> ${job.error}</p>` : ""}
    <div class="detail-actions">
      ${viewerUrl ? `<a class="button-link" href="${viewerUrl}" target="_blank">Open Viewer</a>` : ""}
      ${resultDir ? `<span class="subtle">Result Dir: ${resultDir}</span>` : ""}
    </div>
    <div class="summary-box">${JSON.stringify(job.summary || {}, null, 2)}</div>
    <div class="log-box">${(job.logs || []).join("\n")}</div>
    ${artifacts ? `<ul>${artifacts}</ul>` : ""}
  `;
  const summaryBox = detail.querySelector(".summary-box");
  const logBox = detail.querySelector(".log-box");
  if (summaryBox && state.jobDetailScroll.summary) {
    if (state.jobDetailScroll.summary.atBottom) {
      summaryBox.scrollTop = summaryBox.scrollHeight;
    } else {
      summaryBox.scrollTop = state.jobDetailScroll.summary.top;
    }
  }
  if (logBox && state.jobDetailScroll.logs) {
    if (state.jobDetailScroll.logs.atBottom) {
      logBox.scrollTop = logBox.scrollHeight;
    } else {
      logBox.scrollTop = state.jobDetailScroll.logs.top;
    }
  }
}

async function loadPlugins() {
  state.plugins = await fetchJson("/api/plugins");
  renderPlugins();
  renderPluginDetail();
}

async function loadJobs() {
  state.jobs = await fetchJson("/api/jobs");
  renderJobs();
  renderJobDetail();
}

function installHandlers() {
  document.getElementById("refresh-btn").onclick = loadPlugins;
  document.getElementById("job-refresh-btn").onclick = loadJobs;
  document.getElementById("job-panel-refresh").onclick = loadJobs;
}

async function boot() {
  installHandlers();
  await loadPlugins();
  await loadJobs();
  state.pollHandle = setInterval(loadJobs, 1500);
}

boot().catch((error) => {
  alert(`Failed to boot whl-toolbox: ${error}`);
});

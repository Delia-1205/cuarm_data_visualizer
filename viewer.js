/* global Plotly */
(function () {
  "use strict";

  /** Fallback when log_schema.json 未加载；须与 log_utils.ipp::writeToLogFile 一致 */
  const FALLBACK_SCHEMA = {
    version: 1,
    meta: [
      "running_cost",
      "robotics_total_cost",
      "hardware_communication_cost",
      "communication_cost",
      "motor_communication_cost",
      "gripper_communication_cost",
      "button_communication_cost",
      "actuator_mode",
      "control_strategy",
      "branch_state",
      "system_state",
      "plan_result",
      "system_diagnostic_flags",
      "button_state",
      "simulation",
      "target_type",
    ],
    per_arm: [
      {
        kind: "per_joint_interleaved",
        fields: [
          "motor_target",
          "joint_position",
          "joint_velocity",
          "joint_acceleration",
          "joint_torque",
          "motor_position",
          "motor_velocity",
          "motor_torque",
        ],
      },
      {
        kind: "fixed",
        name: "tool_pose",
        labels: ["x", "y", "z", "qw", "qx", "qy", "qz"],
      },
      { kind: "per_joint", field: "target_before_interpolation" },
      { kind: "per_joint", field: "sliced_tar" },
      { kind: "per_joint", field: "pid_cmd" },
      { kind: "per_joint", field: "guard_pos_cmd" },
      { kind: "per_joint", field: "sat_cmd" },
      { kind: "per_joint", field: "jump_cmd" },
      {
        kind: "fixed",
        name: "task_target_before_interpolation",
        labels: ["x", "y", "z", "rx", "ry", "rz"],
      },
      {
        kind: "fixed",
        name: "task_sliced_tar",
        labels: ["x", "y", "z", "rx", "ry", "rz"],
      },
      {
        kind: "fixed",
        name: "task_pose",
        labels: ["x", "y", "z", "rx", "ry", "rz"],
      },
      { kind: "per_joint", field: "joint_gravity" },
      { kind: "per_joint", field: "friction" },
      { kind: "per_joint", field: "gravity_kd_effect" },
      { kind: "per_joint", field: "filtered_joint_vel" },
      { kind: "per_joint", field: "arm_diagnostic_flags" },
    ],
    per_gripper: [
      { kind: "per_joint", field: "gripper_target" },
      { kind: "per_joint", field: "gripper_position" },
      { kind: "per_joint", field: "gripper_diagnostic_flags" },
    ],
    categories: {
      control: [
        "motor_target",
        "target_before_interpolation",
        "sliced_tar",
        "pid_cmd",
        "guard_pos_cmd",
        "sat_cmd",
        "jump_cmd",
        "task_target_before_interpolation",
        "task_sliced_tar",
        "gripper_target",
      ],
      state: [
        "joint_position",
        "joint_velocity",
        "joint_acceleration",
        "motor_position",
        "motor_velocity",
        "tool_pose",
        "task_pose",
        "filtered_joint_vel",
        "gripper_position",
      ],
      mechanics: [
        "joint_torque",
        "motor_torque",
        "joint_gravity",
        "friction",
        "gravity_kd_effect",
      ],
    },
  };

  var logSchema = FALLBACK_SCHEMA;

  function categorySetsFromSchema(schema) {
    const cats = (schema && schema.categories) || {};
    return {
      control: new Set(cats.control || []),
      state: new Set(cats.state || []),
      mechanics: new Set(cats.mechanics || []),
    };
  }

  function fieldCategory(fieldName, schema) {
    if (fieldName.indexOf("gripper_") === 0) return "gripper";
    if (fieldName.indexOf("diagnostic") !== -1) return "meta";
    const sets = categorySetsFromSchema(schema);
    const base = fieldName.replace(/_(x|y|z|qw|qx|qy|qz|rx|ry|rz)$/, "");
    if (sets.control.has(fieldName) || sets.control.has(base)) return "control";
    if (sets.state.has(fieldName) || sets.state.has(base)) return "state";
    if (sets.mechanics.has(fieldName) || sets.mechanics.has(base)) return "mechanics";
    return "mechanics";
  }

  function expandArmBlock(block, arm, jointCount, schema, cols, idxRef) {
    if (block.kind === "per_joint_interleaved") {
      for (let j = 0; j < jointCount; j++) {
        for (let f = 0; f < block.fields.length; f++) {
          const fname = block.fields[f];
          cols.push({
            index: idxRef.i++,
            key: "Arm" + arm + ".J" + (j + 1) + "." + fname,
            category: fieldCategory(fname, schema),
          });
        }
      }
      return;
    }
    if (block.kind === "per_joint") {
      for (let j = 0; j < jointCount; j++) {
        cols.push({
          index: idxRef.i++,
          key: "Arm" + arm + ".J" + (j + 1) + "." + block.field,
          category: fieldCategory(block.field, schema),
        });
      }
      return;
    }
    if (block.kind === "fixed") {
      const labels = block.labels || [];
      for (let k = 0; k < labels.length; k++) {
        const fname = block.name + "_" + labels[k];
        cols.push({
          index: idxRef.i++,
          key: "Arm" + arm + "." + fname,
          category: fieldCategory(block.name, schema),
        });
      }
    }
  }

  function buildSeriesColumnsFromSchema(s, schema) {
    const sch = schema || logSchema;
    const cols = [];
    const idxRef = { i: 0 };
    const meta = sch.meta || [];
    for (let i = 0; i < meta.length; i++) {
      cols.push({ index: idxRef.i++, key: "Meta." + meta[i], category: "meta" });
    }
    for (let arm = 0; arm < s.ArmSize; arm++) {
      const jn = s.JointSize[arm] || 0;
      const blocks = sch.per_arm || [];
      for (let b = 0; b < blocks.length; b++) {
        expandArmBlock(blocks[b], arm, jn, sch, cols, idxRef);
      }
    }
    const gBlocks = sch.per_gripper || [];
    for (let g = 0; g < s.GripperSize; g++) {
      const gj = s.GripperJointSize[g] || 0;
      for (let b = 0; b < gBlocks.length; b++) {
        const block = gBlocks[b];
        for (let j = 0; j < gj; j++) {
          cols.push({
            index: idxRef.i++,
            key: "Gripper" + (g + 1) + ".J" + (j + 1) + "." + block.field,
            category: "gripper",
          });
        }
      }
    }
    return cols;
  }

  function buildSeriesColumnsFromNames(columnNames, schema) {
    const sch = schema || logSchema;
    const cols = [];
    for (let i = 0; i < columnNames.length; i++) {
      const key = columnNames[i];
      let category = "mechanics";
      if (key.indexOf("Meta.") === 0) category = "meta";
      else if (key.indexOf("Gripper") === 0) category = "gripper";
      else {
        const tail = key.slice(key.lastIndexOf(".") + 1);
        category = fieldCategory(tail, sch);
      }
      cols.push({ index: i, key: key, category: category });
    }
    return cols;
  }

  function expectedColumnCount(s) {
    return buildSeriesColumns(s).length;
  }

  function buildSeriesColumns(s) {
    if (s && Array.isArray(s.columns) && s.columns.length) {
      return buildSeriesColumnsFromNames(s.columns, logSchema);
    }
    return buildSeriesColumnsFromSchema(s, logSchema);
  }

  function fillUniformTime(time, rowCount, sampleT) {
    for (let r = 0; r < rowCount; r++) {
      time[r] = sampleT > 0 ? r * sampleT : r;
    }
    const last = rowCount ? time[rowCount - 1] : 0;
    return {
      time: time,
      timeSource: sampleT > 0 ? "sample_time" : "index",
      durationS: last,
    };
  }

  /** 默认用 running_cost（微秒）累积；仅当用户点「应用」且填写了 sample_time 时用固定步长。 */
  function buildTimeAxis(seriesMeta, columns, rowCount, sampleT, preferSampleTime) {
    const time = new Float32Array(rowCount);
    if (preferSampleTime && sampleT > 0) {
      return fillUniformTime(time, rowCount, sampleT);
    }
    let costIdx = -1;
    for (let i = 0; i < seriesMeta.length; i++) {
      if (seriesMeta[i].key === "Meta.running_cost") {
        costIdx = i;
        break;
      }
    }
    if (costIdx >= 0 && columns[costIdx]) {
      const cost = columns[costIdx];
      let usable = 0;
      for (let r = 0; r < rowCount; r++) {
        if (cost[r] > 0 && Number.isFinite(cost[r])) usable++;
      }
      if (usable > 0) {
        let acc = 0;
        time[0] = 0;
        for (let r = 1; r < rowCount; r++) {
          const dtUs = cost[r - 1] > 0 && Number.isFinite(cost[r - 1]) ? cost[r - 1] : 0;
          acc += dtUs * 1e-6;
          time[r] = acc;
        }
        return { time: time, timeSource: "running_cost", durationS: acc };
      }
    }
    return fillUniformTime(time, rowCount, sampleT);
  }

  function parseLineNumbers(line) {
    const t = line.trim();
    if (!t) return null;
    const parts = t.split(/\s+/);
    const out = [];
    for (let i = 0; i < parts.length; i++) {
      const v = Number(parts[i]);
      if (!Number.isFinite(v)) return null;
      out.push(v);
    }
    return out;
  }

  function parseLogText(text, structure, onProgress) {
    return new Promise(function (resolve, reject) {
      const lines = text.split("\n");
      const seriesMeta = buildSeriesColumns(structure);
      const expected = expectedColumnCount(structure);
      const validRows = [];
      let skipped = 0;
      let firstBad = null;

      for (let i = 0; i < lines.length; i++) {
        if (!lines[i].trim()) continue;
        const nums = parseLineNumbers(lines[i]);
        if (!nums || nums.length !== expected) {
          skipped++;
          if (!firstBad) {
            firstBad = {
              line: i + 1,
              cols: nums ? nums.length : 0,
            };
          }
          continue;
        }
        validRows.push(nums);
      }

      const rowCount = validRows.length;
      if (rowCount === 0) {
        reject(
          new Error(
            "没有完整数据行（期望每行 " +
              expected +
              " 列" +
              (firstBad
                ? "；首个问题行 #" + firstBad.line + " 为 " + firstBad.cols + " 列"
                : "") +
              "）"
          )
        );
        return;
      }

      const nCols = expected;
      const columns = [];
      for (let c = 0; c < nCols; c++) columns.push(new Float32Array(rowCount));

      let r = 0;
      const chunk = 2000;

      function step() {
        try {
          const end = Math.min(r + chunk, rowCount);
          for (; r < end; r++) {
            const nums = validRows[r];
            for (let c = 0; c < nCols; c++) columns[c][r] = nums[c];
          }
          if (onProgress) onProgress(rowCount ? r / rowCount : 1);
          if (r < rowCount) {
            setTimeout(step, 0);
          } else {
            const seriesKeys = seriesMeta.map(function (m) {
              return m.key;
            });
            const keyToCol = new Map();
            for (let i = 0; i < seriesKeys.length; i++) keyToCol.set(seriesKeys[i], i);
            const axis = buildTimeAxis(
              seriesMeta,
              columns,
              rowCount,
              structure.sample_time || 0,
              !!structure.use_sample_time_axis
            );
            resolve({
              structure: structure,
              time: axis.time,
              timeSource: axis.timeSource,
              durationS: axis.durationS,
              columns: columns,
              seriesMeta: seriesMeta,
              seriesKeys: seriesKeys,
              keyToCol: keyToCol,
              rowCount: rowCount,
              skippedRows: skipped,
              firstBadLine: firstBad,
            });
          }
        } catch (e) {
          reject(e);
        }
      }
      step();
    });
  }

  function lttb(x, y, threshold) {
    const n = Math.min(x.length, y.length);
    if (threshold >= n || threshold <= 2) {
      const rx = [];
      const ry = [];
      for (let i = 0; i < n; i++) {
        rx.push(x[i]);
        ry.push(y[i]);
      }
      return { x: rx, y: ry };
    }
    const sampledX = [];
    const sampledY = [];
    const bucketSize = (n - 2) / (threshold - 2);
    let a = 0;
    sampledX.push(x[a]);
    sampledY.push(y[a]);
    for (let i = 0; i < threshold - 2; i++) {
      const rangeStart = Math.floor((i + 1) * bucketSize) + 1;
      const rangeEnd = Math.floor((i + 2) * bucketSize) + 1;
      const end = Math.min(rangeEnd, n);
      let avgX = 0;
      let avgY = 0;
      let count = 0;
      for (let j = rangeStart; j < end; j++) {
        avgX += x[j];
        avgY += y[j];
        count++;
      }
      avgX /= count;
      avgY /= count;
      const rangeTo = Math.min(rangeStart + Math.floor(bucketSize), n - 1);
      let maxArea = -1;
      let maxIdx = rangeStart;
      const ax = x[a];
      const ay = y[a];
      for (let j = rangeStart; j <= rangeTo; j++) {
        const area = Math.abs((ax - avgX) * (y[j] - ay) - (ax - x[j]) * (avgY - ay));
        if (area > maxArea) {
          maxArea = area;
          maxIdx = j;
        }
      }
      sampledX.push(x[maxIdx]);
      sampledY.push(y[maxIdx]);
      a = maxIdx;
    }
    sampledX.push(x[n - 1]);
    sampledY.push(y[n - 1]);
    return { x: sampledX, y: sampledY };
  }

  const PALETTE = [
    "#7aa2f7",
    "#bb9af7",
    "#7dcfff",
    "#9ece6a",
    "#e0af68",
    "#f7768e",
    "#ff9e64",
    "#b4f9f8",
  ];
  const MAX_POINTS = 6000;
  const LS_S = "cuarm_logviz_structure_v1";
  const LS_K = "cuarm_logviz_selected_v1";

  function $(id) {
    return document.getElementById(id);
  }

  var rawText = null;
  var parsed = null;
  var selected = new Set();
  var fileName = "chart";
  var loadedColumns = null;
  var useSampleTimeAxis = false;
  var cat = { meta: true, control: true, state: true, mechanics: true, gripper: true };

  function applyStructureToForm(s) {
    if (!s) return;
    if (s.JointSize) $("joints").value = s.JointSize.join(", ");
    if (s.GripperJointSize) {
      $("grips").value = s.GripperJointSize.length ? s.GripperJointSize.join(", ") : "";
    } else if (s.GripperSize === 0) {
      $("grips").value = "";
    }
    if (s.sample_time !== undefined && s.sample_time !== null) {
      $("sampleT").value = String(s.sample_time);
    }
  }

  function loadStructureForm() {
    try {
      const j = localStorage.getItem(LS_S);
      if (!j) return;
      applyStructureToForm(JSON.parse(j));
    } catch (_) {}
  }

  /** 接受 robot_structure / log_*.structure.json（可含 columns） */
  function normalizeStructureJson(j) {
    const s = {};
    if (Array.isArray(j.JointSize) && j.JointSize.length) {
      s.JointSize = j.JointSize.map(Number);
      s.ArmSize = j.ArmSize != null ? Number(j.ArmSize) : s.JointSize.length;
    } else if (j.ArmSize != null && Array.isArray(j.arm_joint_size)) {
      s.ArmSize = Number(j.ArmSize);
      s.JointSize = j.arm_joint_size.map(Number);
    } else {
      throw new Error("结构 JSON 缺少 JointSize");
    }
    if (Array.isArray(j.GripperJointSize)) {
      s.GripperJointSize = j.GripperJointSize.map(Number).filter(function (n) {
        return n > 0;
      });
      s.GripperSize = j.GripperSize != null ? Number(j.GripperSize) : s.GripperJointSize.length;
    } else if (Array.isArray(j.gripper_joint_size)) {
      s.GripperJointSize = j.gripper_joint_size.map(Number).filter(function (n) {
        return n > 0;
      });
      s.GripperSize =
        j.GripperSize != null ? Number(j.GripperSize) : s.GripperJointSize.length;
    } else {
      s.GripperJointSize = [];
      s.GripperSize = 0;
    }
    if (j.sample_time !== undefined && j.sample_time !== null && Number.isFinite(Number(j.sample_time))) {
      s.sample_time = Number(j.sample_time);
    }
    if (Array.isArray(j.columns) && j.columns.length) {
      s.columns = j.columns.map(String);
    }
    return s;
  }

  function saveStructureForm(s) {
    try {
      localStorage.setItem(LS_S, JSON.stringify(s));
    } catch (_) {}
  }

  function loadSelectedKeys() {
    try {
      const j = localStorage.getItem(LS_K);
      if (!j) return;
      const arr = JSON.parse(j);
      if (Array.isArray(arr))
        arr.forEach(function (k) {
          selected.add(k);
        });
    } catch (_) {}
  }

  function saveSelectedKeys() {
    try {
      localStorage.setItem(LS_K, JSON.stringify(Array.from(selected)));
    } catch (_) {}
  }

  function readStructureFromForm() {
    const joints = $("joints")
      .value.split(/[,，]/)
      .map(function (s) {
        return Number(s.trim());
      })
      .filter(function (n) {
        return n > 0 && Number.isFinite(n);
      });
    if (joints.length === 0) throw new Error("至少填写一个关节数");
    const griPart = $("grips").value.trim();
    const grips =
      griPart === ""
        ? []
        : griPart
            .split(/[,，]/)
            .map(function (s) {
              return Number(s.trim());
            })
            .filter(function (n) {
              return n > 0 && Number.isFinite(n);
            });
    const stRaw = $("sampleT").value.trim();
    var sample_time = stRaw === "" ? undefined : Number(stRaw);
    const s = {
      ArmSize: joints.length,
      JointSize: joints,
      GripperSize: grips.length,
      GripperJointSize: grips,
    };
    if (sample_time !== undefined && Number.isFinite(sample_time)) s.sample_time = sample_time;
    return s;
  }

  function syncSelectionToParsed() {
    if (!parsed) return;
    const valid = new Set(parsed.seriesKeys);
    const kept = Array.from(selected).filter(function (k) {
      return valid.has(k);
    });
    selected.clear();
    if (kept.length) {
      kept.forEach(function (k) {
        selected.add(k);
      });
    } else {
      const def = parsed.seriesKeys
        .filter(function (k) {
          return k.indexOf(".joint_position") !== -1;
        })
        .slice(0, 4);
      const pick = def.length ? def : parsed.seriesKeys.slice(0, 3);
      pick.forEach(function (k) {
        selected.add(k);
      });
    }
    saveSelectedKeys();
  }

  function setProgress(f) {
    const p = $("progress");
    const b = p.querySelector(".bar");
    if (f <= 0 || f >= 1) {
      if (f >= 1) {
        p.hidden = true;
        b.style.width = "0%";
      }
    } else {
      p.hidden = false;
      b.style.width = Math.round(f * 100) + "%";
    }
  }

  function setErr(msg) {
    const e = $("err");
    if (msg) {
      e.textContent = msg;
      e.hidden = false;
    } else {
      e.hidden = true;
    }
  }

  function groupLabel(key) {
    if (key.indexOf("Meta.") === 0) return "Meta";
    const i = key.lastIndexOf(".");
    return i > 0 ? key.slice(0, i) : key;
  }

  function fieldTail(key) {
    const i = key.lastIndexOf(".");
    return i >= 0 ? key.slice(i + 1) : key;
  }

  function visibleMeta() {
    if (!parsed) return [];
    const q = $("search").value.trim().toLowerCase();
    return parsed.seriesMeta.filter(function (m) {
      if (!cat[m.category]) return false;
      if (!q) return true;
      return m.key.toLowerCase().indexOf(q) !== -1;
    });
  }

  function resizePlot() {
    const gd = $("plot");
    if (!gd || typeof Plotly === "undefined") return;
    try {
      Plotly.Plots.resize(gd);
    } catch (_) {}
  }

  function renderTree() {
    const tree = $("tree");
    tree.innerHTML = "";
    if (!parsed) {
      tree.textContent = "请先加载 TXT";
      $("selCount").textContent = "";
      return;
    }
    const vis = visibleMeta();
    const groups = new Map();
    for (let i = 0; i < vis.length; i++) {
      const m = vis[i];
      const g = groupLabel(m.key);
      if (!groups.has(g)) groups.set(g, []);
      groups.get(g).push(m);
    }
    const keys = Array.from(groups.keys()).sort();
    for (let gi = 0; gi < keys.length; gi++) {
      const gk = keys[gi];
      const items = groups.get(gk);
      const det = document.createElement("details");
      det.open = true;
      const sum = document.createElement("summary");
      sum.textContent = gk;
      det.appendChild(sum);
      const ul = document.createElement("ul");
      for (let i = 0; i < items.length; i++) {
        const m = items[i];
        const li = document.createElement("li");
        const lab = document.createElement("label");
        const cb = document.createElement("input");
        cb.type = "checkbox";
        cb.checked = selected.has(m.key);
        cb.addEventListener("change", function () {
          if (cb.checked) selected.add(m.key);
          else selected.delete(m.key);
          saveSelectedKeys();
          redrawPlot();
          $("selCount").textContent = "已选 " + selected.size + " 条曲线";
        });
        lab.appendChild(cb);
        const sp = document.createElement("span");
        sp.textContent = fieldTail(m.key);
        lab.appendChild(sp);
        li.appendChild(lab);
        ul.appendChild(li);
      }
      det.appendChild(ul);
      tree.appendChild(det);
    }
    $("selCount").textContent = "已选 " + selected.size + " 条曲线";
  }

  function redrawPlot() {
    const gd = $("plot");
    if (typeof Plotly === "undefined") return;
    if (!parsed) {
      try {
        Plotly.purge(gd);
      } catch (_) {}
      return;
    }
    const keys = Array.from(selected).sort();
    const traces = [];
    for (let si = 0; si < keys.length; si++) {
      const key = keys[si];
      const col = parsed.keyToCol.get(key);
      if (col === undefined) continue;
      const ds = lttb(parsed.time, parsed.columns[col], MAX_POINTS);
      traces.push({
        type: "scattergl",
        mode: "lines",
        name: key,
        x: ds.x,
        y: ds.y,
        line: { width: 1.5, color: PALETTE[si % PALETTE.length] },
        hovertemplate: "%{fullData.name}<br>t=%{x:.6~f}<br>y=%{y:.6~f}<extra></extra>",
      });
    }
    const timeLabel =
      parsed.timeSource === "running_cost"
        ? "t (s) · running_cost cumulative"
        : parsed.timeSource === "sample_time"
          ? "t (s) · sample_time"
          : "sample index";
    const layout = {
      autosize: true,
      paper_bgcolor: "#16161e",
      plot_bgcolor: "#1a1b26",
      font: { color: "#a9b1d6", size: 12 },
      xaxis: {
        title: { text: timeLabel },
        gridcolor: "#2b3040",
      },
      yaxis: { title: { text: "value" }, gridcolor: "#2b3040" },
      showlegend: true,
      legend: { orientation: "h", y: 1.06, bgcolor: "rgba(22,22,30,0.85)" },
      margin: { t: 48, r: 16, b: 44, l: 56 },
      hovermode: "x unified",
    };
    const config = {
      responsive: true,
      scrollZoom: true,
      displaylogo: false,
      modeBarButtonsToRemove: ["lasso2d", "select2d"],
    };
    Plotly.react(gd, traces, layout, config);
    requestAnimationFrame(function () {
      requestAnimationFrame(resizePlot);
    });
  }

  function parseNow() {
    if (!rawText) return;
    setErr(null);
    var s;
    try {
      s = readStructureFromForm();
      if (loadedColumns && loadedColumns.length) s.columns = loadedColumns;
      s.use_sample_time_axis = useSampleTimeAxis;
      saveStructureForm(s);
    } catch (e) {
      setErr(e.message);
      return;
    }
    const nExpect = expectedColumnCount(s);
    $("colInfo").textContent =
      "期望每行 " +
      nExpect +
      " 列 · ArmSize=" +
      s.ArmSize +
      " · GripperSize=" +
      s.GripperSize +
      (s.columns ? " · schema=sidecar" : " · schema=log_schema");
    setProgress(0.01);
    parseLogText(rawText, s, function (f) {
      setProgress(f);
    })
      .then(function (p) {
        parsed = p;
        setProgress(1);
        var timeNote = "";
        if (p.timeSource === "running_cost") {
          timeNote =
            " · 时间轴 running_cost cumulative " +
            (p.durationS != null ? p.durationS.toFixed(3) + " s" : "");
        } else if (p.timeSource === "sample_time") {
          timeNote = " · 时间轴 sample_time";
        } else {
          timeNote = " · 时间轴为行号";
        }
        $("stats").textContent =
          p.rowCount.toLocaleString() +
          " 行 · " +
          p.seriesKeys.length +
          " 列" +
          timeNote +
          (p.skippedRows
            ? " · 跳过 " +
              p.skippedRows +
              " 行不完整" +
              (p.firstBadLine ? "（如 #" + p.firstBadLine.line + "）" : "")
            : "");
        syncSelectionToParsed();
        renderTree();
        redrawPlot();
      })
      .catch(function (e) {
        parsed = null;
        setProgress(1);
        setErr(e.message || String(e));
        renderTree();
        redrawPlot();
      });
  }

  function buildChips() {
    const wrap = $("chips");
    const defs = [
      ["meta", "Meta"],
      ["control", "控制"],
      ["state", "状态"],
      ["mechanics", "力学"],
      ["gripper", "夹爪"],
    ];
    wrap.innerHTML = "";
    for (let i = 0; i < defs.length; i++) {
      const id = defs[i][0];
      const lab = document.createElement("label");
      lab.className = "chip";
      const cb = document.createElement("input");
      cb.type = "checkbox";
      cb.checked = true;
      cb.addEventListener("change", function () {
        cat[id] = cb.checked;
        renderTree();
      });
      lab.appendChild(cb);
      lab.appendChild(document.createTextNode(defs[i][1]));
      wrap.appendChild(lab);
    }
  }

  $("txtFile").addEventListener("change", function (ev) {
    const f = ev.target.files[0];
    if (!f) return;
    fileName = f.name.replace(/\.[^.]+$/, "") || "chart";
    const r = new FileReader();
    r.onload = function () {
      rawText = String(r.result);
      useSampleTimeAxis = false;
      parseNow();
    };
    r.readAsText(f);
  });

  $("jsonFile").addEventListener("change", function (ev) {
    const f = ev.target.files[0];
    if (!f) return;
    const r = new FileReader();
    r.onload = function () {
      try {
        const j = JSON.parse(String(r.result));
        const root = j.structure && typeof j.structure === "object" ? j.structure : j;
        const s = normalizeStructureJson(root);
        applyStructureToForm(s);
        loadedColumns = s.columns || null;
        saveStructureForm(s);
        setErr(null);
      } catch (e) {
        setErr("JSON: " + e.message);
      }
      if (rawText) parseNow();
    };
    r.readAsText(f);
  });

  $("btnApply").addEventListener("click", function () {
    const st = Number($("sampleT").value.trim());
    useSampleTimeAxis = Number.isFinite(st) && st > 0;
    parseNow();
  });

  function openHelp() {
    const modal = $("helpModal");
    const frame = $("helpFrame");
    if (!frame.getAttribute("src")) frame.setAttribute("src", "fields.html");
    modal.hidden = false;
  }

  function closeHelp() {
    $("helpModal").hidden = true;
  }

  $("btnHelp").addEventListener("click", openHelp);
  $("helpModal").addEventListener("click", function (ev) {
    if (ev.target && ev.target.hasAttribute("data-close-help")) closeHelp();
  });
  document.addEventListener("keydown", function (ev) {
    if (ev.key === "Escape" && !$("helpModal").hidden) closeHelp();
  });

  $("search").addEventListener("input", function () {
    renderTree();
  });

  $("allVis").addEventListener("click", function () {
    if (!parsed) return;
    visibleMeta().forEach(function (m) {
      selected.add(m.key);
    });
    saveSelectedKeys();
    renderTree();
    redrawPlot();
  });

  $("clearSel").addEventListener("click", function () {
    selected.clear();
    saveSelectedKeys();
    renderTree();
    redrawPlot();
  });

  $("selPos").addEventListener("click", function () {
    if (!parsed) return;
    selected.clear();
    visibleMeta().forEach(function (m) {
      if (m.key.indexOf(".joint_position") !== -1) selected.add(m.key);
    });
    saveSelectedKeys();
    renderTree();
    redrawPlot();
  });

  $("selTq").addEventListener("click", function () {
    if (!parsed) return;
    selected.clear();
    visibleMeta().forEach(function (m) {
      const k = m.key;
      if (
        k.indexOf("motor_torque") !== -1 ||
        k.indexOf("joint_torque") !== -1 ||
        k.indexOf("torque_sensor") !== -1
      ) {
        selected.add(k);
      }
    });
    saveSelectedKeys();
    renderTree();
    redrawPlot();
  });

  $("btnPng").addEventListener("click", function () {
    if (!parsed) return;
    Plotly.downloadImage($("plot"), { format: "png", filename: fileName, width: 1200, height: 720 });
  });

  $("btnCfg").addEventListener("click", function () {
    var s;
    try {
      s = readStructureFromForm();
    } catch (e) {
      setErr(e.message);
      return;
    }
    const blob = new Blob(
      [JSON.stringify({ structure: s, selected: Array.from(selected).sort() }, null, 2)],
      { type: "application/json" }
    );
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = "cuarm_logviz_config.json";
    a.click();
    URL.revokeObjectURL(a.href);
  });

  window.addEventListener("resize", resizePlot);

  var plotWrap = document.querySelector(".plot-wrap");
  if (plotWrap && typeof ResizeObserver !== "undefined") {
    new ResizeObserver(resizePlot).observe(plotWrap);
  }

  function refreshColInfo() {
    try {
      const s = readStructureFromForm();
      if (loadedColumns && loadedColumns.length) s.columns = loadedColumns;
      $("colInfo").textContent =
        "期望每行 " +
        expectedColumnCount(s) +
        " 列 · ArmSize=" +
        s.ArmSize +
        " · GripperSize=" +
        s.GripperSize;
    } catch (_) {
      $("colInfo").textContent = "";
    }
  }

  buildChips();
  loadStructureForm();
  loadSelectedKeys();
  refreshColInfo();

  fetch("log_schema.json")
    .then(function (r) {
      if (!r.ok) throw new Error("HTTP " + r.status);
      return r.json();
    })
    .then(function (j) {
      if (j && Array.isArray(j.meta) && Array.isArray(j.per_arm)) {
        logSchema = j;
        refreshColInfo();
      }
    })
    .catch(function () {
      /* 使用内置 FALLBACK_SCHEMA */
    });
})();

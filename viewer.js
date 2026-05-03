/* global Plotly */
(function () {
  "use strict";

  const META_FIELD_NAMES = [
    "running_cost",
    "robotics_total_cost",
    "communication_cost",
    "hardware_communication_cost",
    "robotics_setup_cost",
    "robotics_Null_cost",
    "robotics_FK_cost",
    "robotics_IK_cost",
    "robotics_ID_cost",
    "robotics_simulation_addition_cost",
    "planner_mode",
    "planner_status",
    "command_mode",
  ];

  const JOINT_FIELD_NAMES = [
    "sliced_tar",
    "motor_target",
    "joint_position",
    "joint_velocity",
    "joint_acceleration",
    "motor_position",
    "motor_velocity",
    "motor_torque",
    "target_before_interpolation",
    "motor_raw_position",
    "joint_gravity",
    "gravity",
    "sat_motor_cmd",
    "joint_impedance",
    "impedance",
    "joint_torque",
    "torque_sensor_raw",
    "torque_sensor",
    "encoder_sensor_raw",
    "friction",
    "gravity_kd_effect",
    "filtered_joint_vel",
  ];

  const CONTROL = new Set([
    "sliced_tar",
    "motor_target",
    "target_before_interpolation",
    "sat_motor_cmd",
    "gripper_command",
    "gripper_act_set_command",
  ]);
  const STATE = new Set([
    "joint_position",
    "joint_velocity",
    "joint_acceleration",
    "motor_position",
    "motor_velocity",
    "motor_raw_position",
    "encoder_sensor_raw",
    "filtered_joint_vel",
    "gripper_position",
  ]);

  function jointFieldCategory(name) {
    if (CONTROL.has(name)) return "control";
    if (STATE.has(name)) return "state";
    return "mechanics";
  }

  function expectedColumnCount(s) {
    let n = META_FIELD_NAMES.length;
    for (let a = 0; a < s.ArmSize; a++) {
      const jn = s.JointSize[a] || 0;
      n += jn * JOINT_FIELD_NAMES.length;
    }
    for (let g = 0; g < s.GripperSize; g++) {
      const gj = s.GripperJointSize[g] || 0;
      n += gj * 3;
    }
    return n;
  }

  function buildSeriesColumns(s) {
    const cols = [];
    let idx = 0;
    for (let i = 0; i < META_FIELD_NAMES.length; i++) {
      cols.push({ index: idx++, key: "Meta." + META_FIELD_NAMES[i], category: "meta" });
    }
    for (let arm = 0; arm < s.ArmSize; arm++) {
      const jn = s.JointSize[arm] || 0;
      for (let j = 0; j < jn; j++) {
        for (let f = 0; f < JOINT_FIELD_NAMES.length; f++) {
          const fname = JOINT_FIELD_NAMES[f];
          cols.push({
            index: idx++,
            key: "Arm" + arm + ".J" + (j + 1) + "." + fname,
            category: jointFieldCategory(fname),
          });
        }
      }
    }
    for (let pass = 0; pass < 3; pass++) {
      const suf =
        pass === 0
          ? "gripper_command"
          : pass === 1
            ? "gripper_act_set_command"
            : "gripper_position";
      for (let g = 0; g < s.GripperSize; g++) {
        const gj = s.GripperJointSize[g] || 0;
        for (let j = 0; j < gj; j++) {
          cols.push({
            index: idx++,
            key: "Gripper" + (g + 1) + ".J" + (j + 1) + "." + suf,
            category: "gripper",
          });
        }
      }
    }
    return cols;
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
      const nonempty = [];
      for (let i = 0; i < lines.length; i++) {
        if (lines[i].trim()) nonempty.push(lines[i]);
      }
      const rowCount = nonempty.length;
      if (rowCount === 0) {
        reject(new Error("空文件或没有数据行"));
        return;
      }
      const seriesMeta = buildSeriesColumns(structure);
      const expected = expectedColumnCount(structure);
      const first = parseLineNumbers(nonempty[0]);
      if (!first || first.length !== expected) {
        reject(
          new Error(
            "列数不匹配: 首行 " + (first ? first.length : 0) + " 个值，期望 " + expected + "。"
          )
        );
        return;
      }
      const nCols = expected;
      const columns = [];
      for (let c = 0; c < nCols; c++) columns.push(new Float32Array(rowCount));
      const sampleT = structure.sample_time || 0;
      const time = new Float32Array(rowCount);
      for (let r = 0; r < rowCount; r++) {
        time[r] = sampleT > 0 ? r * sampleT : r;
      }

      let r = 0;
      const chunk = 2000;

      function step() {
        try {
          const end = Math.min(r + chunk, rowCount);
          for (; r < end; r++) {
            const nums = r === 0 ? first : parseLineNumbers(nonempty[r]);
            if (!nums || nums.length !== expected) {
              throw new Error(
                "第 " + (r + 1) + " 行列数错误: " + (nums ? nums.length : 0) + " != " + expected
              );
            }
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
            resolve({
              structure: structure,
              time: time,
              columns: columns,
              seriesMeta: seriesMeta,
              seriesKeys: seriesKeys,
              keyToCol: keyToCol,
              rowCount: rowCount,
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
  var cat = { meta: true, control: true, state: true, mechanics: true, gripper: true };

  function loadStructureForm() {
    try {
      const j = localStorage.getItem(LS_S);
      if (!j) return;
      const s = JSON.parse(j);
      if (s.JointSize) $("joints").value = s.JointSize.join(", ");
      if (s.GripperJointSize) $("grips").value = s.GripperJointSize.join(", ");
      $("sampleT").value =
        s.sample_time !== undefined && s.sample_time !== null ? String(s.sample_time) : "";
    } catch (_) {}
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
    const sampleT = parsed.structure.sample_time || 0;
    const layout = {
      autosize: true,
      paper_bgcolor: "#16161e",
      plot_bgcolor: "#1a1b26",
      font: { color: "#a9b1d6", size: 12 },
      xaxis: {
        title: { text: sampleT > 0 ? "t (s)" : "sample index" },
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
      saveStructureForm(s);
    } catch (e) {
      setErr(e.message);
      return;
    }
    $("colInfo").textContent =
      "期望每行 " +
      expectedColumnCount(s) +
      " 列 · ArmSize=" +
      s.ArmSize +
      " · GripperSize=" +
      s.GripperSize;
    setProgress(0.01);
    parseLogText(rawText, s, function (f) {
      setProgress(f);
    })
      .then(function (p) {
        parsed = p;
        setProgress(1);
        $("stats").textContent =
          p.rowCount.toLocaleString() + " 行 · " + p.seriesKeys.length + " 列";
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
        if (j.JointSize) $("joints").value = j.JointSize.join(", ");
        if (j.GripperJointSize) $("grips").value = j.GripperJointSize.join(", ");
        $("sampleT").value =
          j.sample_time !== undefined && j.sample_time !== null ? String(j.sample_time) : "";
      } catch (e) {
        setErr("JSON: " + e.message);
      }
      if (rawText) parseNow();
    };
    r.readAsText(f);
  });

  $("btnApply").addEventListener("click", function () {
    parseNow();
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

  buildChips();
  loadStructureForm();
  loadSelectedKeys();
  try {
    const s = readStructureFromForm();
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
})();

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
  var rtControlSchema = null;
  var activeSchemaId = "telemetry";

  function getActiveSchema() {
    return activeSchemaId === "rt_control" && rtControlSchema ? rtControlSchema : logSchema;
  }

  function setActiveSchema(id) {
    activeSchemaId = id === "rt_control" && rtControlSchema ? "rt_control" : "telemetry";
  }

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

  function buildSeriesColumns(s, schema) {
    const sch = schema || getActiveSchema();
    if (s && Array.isArray(s.columns) && s.columns.length) {
      return buildSeriesColumnsFromNames(s.columns, sch);
    }
    return buildSeriesColumnsFromSchema(s, sch);
  }

  function expectedColumnCount(s, schema) {
    return buildSeriesColumns(s, schema).length;
  }

  function sniffTxtColumnCount(text) {
    const lines = text.split("\n");
    let dataStart = lines.length > 0 && /^\s*#/.test(lines[0]) ? 1 : 0;
    for (let i = dataStart; i < lines.length; i++) {
      if (!lines[i].trim()) continue;
      const n = countLineFields(lines[i]);
      if (n > 0) return n;
    }
    return 0;
  }

  function inferRtControlStructure(colCount) {
    if (!rtControlSchema) return null;
    const known = [[7, 2, 7], [7, 7], [6, 6], [6], [7]];
    for (let i = 0; i < known.length; i++) {
      const js = known[i];
      const s = {
        ArmSize: js.length,
        JointSize: js.slice(),
        GripperSize: 0,
        GripperJointSize: [],
      };
      if (expectedColumnCount(s, rtControlSchema) === colCount) return s;
    }
    return null;
  }

  function resolveTxtParsePlan(textOrColCount, formStructure) {
    const colCount =
      typeof textOrColCount === "number" ? textOrColCount : sniffTxtColumnCount(textOrColCount);
    if (!colCount) return { schemaId: activeSchemaId, structure: formStructure };
    if (expectedColumnCount(formStructure, logSchema) === colCount) {
      return { schemaId: "telemetry", structure: formStructure };
    }
    if (rtControlSchema) {
      if (expectedColumnCount(formStructure, rtControlSchema) === colCount) {
        return { schemaId: "rt_control", structure: formStructure };
      }
      const inferred = inferRtControlStructure(colCount);
      if (inferred) return { schemaId: "rt_control", structure: inferred };
    }
    return { schemaId: activeSchemaId, structure: formStructure };
  }

  function schemaLabel() {
    if (rawBinary) return "CTLG binary";
    if (loadedColumns && loadedColumns.length) return "sidecar";
    return activeSchemaId === "rt_control" ? "rt_control" : "telemetry";
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

  function countLineFields(line) {
    let n = 0;
    let i = 0;
    const len = line.length;
    while (i < len) {
      while (i < len && (line.charCodeAt(i) === 32 || line.charCodeAt(i) === 9)) i++;
      if (i >= len) break;
      let j = i;
      while (j < len && line.charCodeAt(j) !== 32 && line.charCodeAt(j) !== 9) j++;
      n++;
      i = j;
    }
    return n;
  }

  function parseLineToColumns(line, expected, columns, row) {
    let i = 0;
    let col = 0;
    const len = line.length;
    while (i < len && col < expected) {
      while (i < len && (line.charCodeAt(i) === 32 || line.charCodeAt(i) === 9)) i++;
      if (i >= len) return false;
      let j = i;
      while (j < len && line.charCodeAt(j) !== 32 && line.charCodeAt(j) !== 9) j++;
      const v = Number(line.slice(i, j));
      if (!Number.isFinite(v)) return false;
      columns[col][row] = v;
      col++;
      i = j;
    }
    return col === expected;
  }

  function finalizeLogParse(structure, seriesMeta, columns, rowCount, skipped, firstBad) {
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
    return {
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
    };
  }

  function allocateColumnBuffers(nCols, rowCount) {
    const columns = new Array(nCols);
    for (let c = 0; c < nCols; c++) columns[c] = new Float32Array(rowCount);
    return columns;
  }

  function trimColumns(columns, rowCount) {
    for (let c = 0; c < columns.length; c++) {
      if (columns[c].length > rowCount) columns[c] = columns[c].subarray(0, rowCount);
    }
  }

  function growColumns(columns, newCap) {
    for (let c = 0; c < columns.length; c++) {
      const old = columns[c];
      if (old.length >= newCap) continue;
      const neu = new Float32Array(newCap);
      neu.set(old);
      columns[c] = neu;
    }
  }

  function readParseOptionsFromForm() {
    const skip = Math.max(0, parseInt($("parseSkip").value, 10) || 0);
    const maxRaw = parseInt($("parseMaxRows").value, 10);
    const maxRows = Number.isFinite(maxRaw) && maxRaw > 0 ? maxRaw : 0;
    const decimate = Math.max(1, parseInt($("parseDecimate").value, 10) || 1);
    const autoDecimate = $("parseAutoDecimate").checked;
    return { skipRows: skip, maxRows: maxRows, decimate: decimate, autoDecimate: autoDecimate };
  }

  function saveParseOptionsForm(opts) {
    try {
      localStorage.setItem(LS_P, JSON.stringify(opts));
    } catch (_) {}
  }

  function loadParseOptionsForm() {
    try {
      const j = localStorage.getItem(LS_P);
      if (!j) return;
      const o = JSON.parse(j);
      if (o.skipRows != null) $("parseSkip").value = String(o.skipRows);
      if (o.maxRows != null && o.maxRows > 0) $("parseMaxRows").value = String(o.maxRows);
      if (o.decimate != null) $("parseDecimate").value = String(o.decimate);
      if (o.autoDecimate != null) $("parseAutoDecimate").checked = !!o.autoDecimate;
    } catch (_) {}
  }

  function shouldUseSinglePass(parseOpts, estimatedRows) {
    if (parseOpts.skipRows > 0) return true;
    if (parseOpts.maxRows > 0) return true;
    if (parseOpts.decimate > 1) return true;
    if (parseOpts.autoDecimate && estimatedRows > DEFAULT_MAX_STORED_ROWS) return true;
    return false;
  }

  function computeParsePlan(parseOpts, estimatedRows, fileSize) {
    const skip = parseOpts.skipRows;
    let decimate = parseOpts.decimate;
    const hardCap = parseOpts.maxRows > 0 ? parseOpts.maxRows : DEFAULT_MAX_STORED_ROWS;
    let autoApplied = false;

    if (parseOpts.autoDecimate && decimate <= 1 && estimatedRows > hardCap) {
      decimate = Math.max(1, Math.ceil(estimatedRows / hardCap));
      autoApplied = true;
    }

    const afterSkip = Math.max(0, estimatedRows - skip);
    let storeCount = Math.ceil(afterSkip / decimate);
    if (parseOpts.maxRows > 0) storeCount = Math.min(storeCount, parseOpts.maxRows);

    return {
      skipRows: skip,
      decimate: decimate,
      maxRows: parseOpts.maxRows,
      storeCount: Math.max(1, storeCount),
      estimatedRows: estimatedRows,
      autoDecimateApplied: autoApplied,
      stopEarly: parseOpts.maxRows > 0,
    };
  }

  function estimateValidRowsFromFile(file, expected) {
    const sampleBytes = Math.min(file.size, 512 * 1024);
    const blob = file.slice(0, sampleBytes);
    let valid = 0;
    let total = 0;
    let firstLine = true;
    return streamFileLines(blob, {
      yieldEvery: 5000,
      onLine: function (line) {
        if (firstLine && /^\s*#/.test(line)) {
          firstLine = false;
          return;
        }
        firstLine = false;
        if (!line.trim()) return;
        total++;
        if (countLineFields(line) === expected) valid++;
      },
    }).then(function () {
      if (!valid) return Math.max(1, Math.floor(file.size / (expected * 8 + 16)));
      const validRatio = valid / Math.max(1, total);
      const avgBytesPerLine = sampleBytes / Math.max(1, total);
      return Math.max(valid, Math.round((file.size / avgBytesPerLine) * validRatio));
    });
  }

  function attachParseMeta(result, plan, sourceRowsSeen, truncated) {
    result.parseMeta = {
      estimatedSourceRows: plan.estimatedRows,
      sourceRowsSeen: sourceRowsSeen,
      skipRows: plan.skipRows,
      decimateStride: plan.decimate,
      autoDecimateApplied: plan.autoDecimateApplied,
      truncated: !!truncated,
    };
    if (plan.decimate > 1 || plan.skipRows > 0) {
      const time = new Float32Array(result.rowCount);
      for (let i = 0; i < result.rowCount; i++) {
        time[i] = plan.skipRows + i * plan.decimate;
      }
      result.time = time;
      result.timeSource = "decimated_index";
      result.durationS = result.rowCount ? time[result.rowCount - 1] : 0;
    }
    return result;
  }

  function parseLogStreamSinglePass(lineSource, structure, plan, onProgress) {
    const seriesMeta = buildSeriesColumns(structure);
    const expected = expectedColumnCount(structure);
    let allocRows = plan.storeCount;
    if (plan.maxRows > 0) allocRows = plan.maxRows;
    else if (plan.autoDecimateApplied) allocRows = Math.min(plan.storeCount, DEFAULT_MAX_STORED_ROWS);
    allocRows = Math.max(1024, Math.ceil(allocRows * 1.05));

    const columns = allocateColumnBuffers(expected, allocRows);
    let validIdx = 0;
    let stored = 0;
    let skipped = 0;
    let firstBad = null;
    let firstLine = true;
    let truncated = false;
    const storeCap = plan.maxRows > 0 ? plan.maxRows : Infinity;

    function handleLine(line, lineNum) {
      if (firstLine && /^\s*#/.test(line)) {
        firstLine = false;
        return;
      }
      firstLine = false;
      if (!line.trim()) return;
      const n = countLineFields(line);
      if (n !== expected) {
        skipped++;
        if (!firstBad) firstBad = { line: lineNum || 0, cols: n };
        return;
      }
      const vi = validIdx++;
      if (vi < plan.skipRows) return;
      if ((vi - plan.skipRows) % plan.decimate !== 0) return;
      if (stored >= storeCap) {
        truncated = true;
        if (plan.stopEarly) return false;
        return;
      }
      if (stored >= columns[0].length) {
        growColumns(columns, Math.ceil(columns[0].length * 1.5));
      }
      parseLineToColumns(line, expected, columns, stored);
      stored++;
    }

    return lineSource(handleLine, onProgress).then(function () {
      if (stored === 0) {
        throw new Error(
          "没有完整数据行（期望每行 " +
            expected +
            " 列" +
            (firstBad ? "；首个问题行 #" + firstBad.line + " 为 " + firstBad.cols + " 列" : "") +
            "）"
        );
      }
      trimColumns(columns, stored);
      const result = finalizeLogParse(structure, seriesMeta, columns, stored, skipped, firstBad);
      return attachParseMeta(result, plan, validIdx, truncated);
    });
  }

  function streamFileLines(file, handlers) {
    handlers = handlers || {};
    const onLine = handlers.onLine;
    const maxLines = handlers.maxLines || Infinity;
    const yieldEvery = handlers.yieldEvery || STREAM_YIELD_LINES;

    return new Promise(function (resolve, reject) {
      if (!file.stream) {
        reject(new Error("浏览器不支持流式读取，请使用 Chrome / Edge"));
        return;
      }
      const reader = file.stream().getReader();
      const decoder = new TextDecoder();
      let buffer = "";
      let lineNum = 0;
      let linesSinceYield = 0;
      let bytesRead = 0;
      const totalBytes = file.size || 1;
      let aborted = false;

      function emitLine(line) {
        lineNum++;
        if (onLine) {
          const keep = onLine(line, lineNum);
          if (keep === false) {
            aborted = true;
            reader.cancel().catch(function () {});
            resolve(lineNum);
            return false;
          }
        }
        linesSinceYield++;
        if (lineNum >= maxLines) {
          aborted = true;
          reader.cancel().catch(function () {});
          resolve(lineNum);
          return false;
        }
        return true;
      }

      function processPending() {
        let nl;
        while (!aborted && (nl = buffer.indexOf("\n")) >= 0 && lineNum < maxLines) {
          let line = buffer.slice(0, nl);
          buffer = buffer.slice(nl + 1);
          if (line.endsWith("\r")) line = line.slice(0, -1);
          if (emitLine(line) === false) return;
        }
      }

      function pump() {
        reader.read().then(function (chunk) {
          if (aborted) return;
          if (chunk.done) {
            if (buffer.length && lineNum < maxLines) emitLine(buffer);
            resolve(lineNum);
            return;
          }
          bytesRead += chunk.value.byteLength;
          buffer += decoder.decode(chunk.value, { stream: true });
          processPending();
          if (aborted) return;
          if (handlers.onProgress) handlers.onProgress(bytesRead / totalBytes);
          if (linesSinceYield >= yieldEvery) {
            linesSinceYield = 0;
            setTimeout(pump, 0);
          } else {
            pump();
          }
        }).catch(reject);
      }
      pump();
    });
  }

  function sniffTxtFile(file) {
    let colCount = 0;
    let firstLine = true;
    return streamFileLines(file, {
      maxLines: 40,
      yieldEvery: 40,
      onLine: function (line) {
        if (firstLine && /^\s*#/.test(line)) {
          firstLine = false;
          return;
        }
        firstLine = false;
        if (!line.trim()) return;
        const n = countLineFields(line);
        if (n > colCount) colCount = n;
      },
    }).then(function () {
      return colCount;
    });
  }

  function parseLogText(text, structure, parseOpts, onProgress) {
    return new Promise(function (resolve, reject) {
      setTimeout(function () {
        let lines;
        try {
          lines = text.split("\n");
        } catch (e) {
          reject(e);
          return;
        }
        let dataStart = 0;
        if (lines.length > 0 && /^\s*#/.test(lines[0])) dataStart = 1;

        const expected = expectedColumnCount(structure);
        let estimatedRows = 0;
        for (let i = dataStart; i < lines.length; i++) {
          if (!lines[i].trim()) continue;
          if (countLineFields(lines[i]) === expected) estimatedRows++;
        }

        const plan = computeParsePlan(parseOpts, estimatedRows, text.length);

        if (shouldUseSinglePass(parseOpts, estimatedRows)) {
          let li = dataStart;
          parseLogStreamSinglePass(
            function (handleLine, prog) {
              return new Promise(function (res, rej) {
                function step() {
                  try {
                    const end = Math.min(li + 12000, lines.length);
                    for (; li < end; li++) handleLine(lines[li], li + 1);
                    if (prog) prog(li / lines.length);
                    if (li < lines.length) setTimeout(step, 0);
                    else res();
                  } catch (e) {
                    rej(e);
                  }
                }
                step();
              });
            },
            structure,
            plan,
            onProgress
          )
            .then(resolve)
            .catch(reject);
          return;
        }

        const seriesMeta = buildSeriesColumns(structure);
        const rowCount = estimatedRows;
        if (rowCount === 0) {
          reject(new Error("没有完整数据行（期望每行 " + expected + " 列）"));
          return;
        }

        const columns = allocateColumnBuffers(expected, rowCount);
        let row = 0;
        li = dataStart;

        function pass2() {
          try {
            const end = Math.min(li + 8000, lines.length);
            for (; li < end; li++) {
              if (!lines[li].trim()) continue;
              if (parseLineToColumns(lines[li], expected, columns, row)) row++;
            }
            if (onProgress) onProgress(0.35 + (0.65 * li) / lines.length);
            if (li < lines.length) setTimeout(pass2, 0);
            else {
              if (row !== rowCount) {
                reject(new Error("解析行数不一致（" + row + " / " + rowCount + "）"));
                return;
              }
              resolve(finalizeLogParse(structure, seriesMeta, columns, rowCount, 0, null));
            }
          } catch (e) {
            reject(e);
          }
        }
        if (onProgress) onProgress(0.35);
        pass2();
      }, 0);
    });
  }

  function hasManualParseOpts(parseOpts) {
    return parseOpts.skipRows > 0 || parseOpts.maxRows > 0 || parseOpts.decimate > 1;
  }

  function parseLogStreamFullFile(file, structure, onProgress) {
    const seriesMeta = buildSeriesColumns(structure);
    const expected = expectedColumnCount(structure);
    const columns = allocateColumnBuffers(expected, 2048);
    let row = 0;
    let skipped = 0;
    let firstBad = null;
    let firstLine = true;

    return streamFileLines(file, {
      onLine: function (line, lineNum) {
        if (firstLine && /^\s*#/.test(line)) {
          firstLine = false;
          return;
        }
        firstLine = false;
        if (!line.trim()) return;
        const n = countLineFields(line);
        if (n !== expected) {
          skipped++;
          if (!firstBad) firstBad = { line: lineNum, cols: n };
          return;
        }
        if (row >= columns[0].length) {
          growColumns(columns, Math.max(row + 512, Math.ceil(columns[0].length * 1.5)));
        }
        parseLineToColumns(line, expected, columns, row);
        row++;
      },
      onProgress: onProgress,
    }).then(function () {
      if (row === 0) {
        throw new Error(
          "没有完整数据行（期望每行 " +
            expected +
            " 列" +
            (firstBad ? "；首个问题行 #" + firstBad.line + " 为 " + firstBad.cols + " 列" : "") +
            "）"
        );
      }
      trimColumns(columns, row);
      return finalizeLogParse(structure, seriesMeta, columns, row, skipped, firstBad);
    });
  }

  function parseLogTextFromFile(file, structure, parseOpts, onProgress) {
    const expected = expectedColumnCount(structure);

    if (
      !hasManualParseOpts(parseOpts) &&
      (!parseOpts.autoDecimate || file.size < AUTO_DECIMATE_ESTIMATE_BYTES)
    ) {
      return parseLogStreamFullFile(file, structure, onProgress);
    }

    return estimateValidRowsFromFile(file, expected).then(function (estimatedRows) {
      const plan = computeParsePlan(parseOpts, estimatedRows, file.size);

      if (!hasManualParseOpts(parseOpts) && !shouldUseSinglePass(parseOpts, estimatedRows)) {
        return parseLogStreamFullFile(file, structure, onProgress);
      }

      if (shouldUseSinglePass(parseOpts, estimatedRows)) {
        if (plan.autoDecimateApplied && onProgress) onProgress(0.02);
        return parseLogStreamSinglePass(
          function (handleLine, prog) {
            return streamFileLines(file, {
              onLine: handleLine,
              onProgress: prog,
            });
          },
          structure,
          plan,
          onProgress
        );
      }

      const seriesMeta = buildSeriesColumns(structure);
      let rowCount = 0;
      let skipped = 0;
      let firstBad = null;
      let firstLine = true;

      return streamFileLines(file, {
        onLine: function (line, lineNum) {
          if (firstLine && /^\s*#/.test(line)) {
            firstLine = false;
            return;
          }
          firstLine = false;
          if (!line.trim()) return;
          const n = countLineFields(line);
          if (n === expected) rowCount++;
          else {
            skipped++;
            if (!firstBad) firstBad = { line: lineNum, cols: n };
          }
        },
        onProgress: function (frac) {
          if (onProgress) onProgress(frac * 0.4);
        },
      }).then(function () {
        if (rowCount === 0) {
          throw new Error(
            "没有完整数据行（期望每行 " +
              expected +
              " 列" +
              (firstBad ? "；首个问题行 #" + firstBad.line + " 为 " + firstBad.cols + " 列" : "") +
              "）"
          );
        }
        const columns = allocateColumnBuffers(expected, rowCount);
        let row = 0;
        firstLine = true;
        return streamFileLines(file, {
          onLine: function (line) {
            if (firstLine && /^\s*#/.test(line)) {
              firstLine = false;
              return;
            }
            firstLine = false;
            if (!line.trim()) return;
            if (parseLineToColumns(line, expected, columns, row)) row++;
          },
          onProgress: function (frac) {
            if (onProgress) onProgress(0.4 + frac * 0.6);
          },
        }).then(function () {
          if (row !== rowCount) {
            throw new Error("解析行数不一致（" + row + " / " + rowCount + "）");
          }
          return finalizeLogParse(structure, seriesMeta, columns, rowCount, skipped, firstBad);
        });
      });
    });
  }

  function parseLogBinary(buffer, structure, onProgress) {
    return TelemetryBinary.parseBinaryTable(buffer, onProgress).then(function (bin) {
      var seriesMeta = buildSeriesColumnsFromNames(bin.viewerColumnNames, logSchema);
      var expected = bin.nCols;
      if (seriesMeta.length !== expected) {
        throw new Error("列名映射异常：期望 " + expected + " 列，得到 " + seriesMeta.length);
      }
      var rowCount = bin.rowCount;
      var nCols = expected;
      var columns = [];
      for (var c = 0; c < nCols; c++) columns.push(new Float32Array(rowCount));
      for (var r = 0; r < rowCount; r++) {
        var nums = bin.rows[r];
        for (var c2 = 0; c2 < nCols; c2++) columns[c2][r] = nums[c2];
      }
      var seriesKeys = seriesMeta.map(function (m) {
        return m.key;
      });
      var keyToCol = new Map();
      for (var i = 0; i < seriesKeys.length; i++) keyToCol.set(seriesKeys[i], i);
      var axis = buildTimeAxis(
        seriesMeta,
        columns,
        rowCount,
        structure.sample_time || 0,
        !!structure.use_sample_time_axis
      );
      return {
        structure: structure,
        time: axis.time,
        timeSource: axis.timeSource,
        durationS: axis.durationS,
        columns: columns,
        seriesMeta: seriesMeta,
        seriesKeys: seriesKeys,
        keyToCol: keyToCol,
        rowCount: rowCount,
        skippedRows: 0,
        firstBadLine: null,
        binaryHeader: bin.header,
      };
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
  const DEFAULT_MAX_STORED_ROWS = 300000;
  /** 低于此体积时 autoDecimate 不可能触发，可跳过行数估计 */
  const AUTO_DECIMATE_ESTIMATE_BYTES = 64 * 1024 * 1024;
  const STREAM_YIELD_LINES = 12000;
  const LS_S = "cuarm_logviz_structure_v1";
  const LS_K = "cuarm_logviz_selected_v1";
  const LS_P = "cuarm_logviz_parse_v1";

  function $(id) {
    return document.getElementById(id);
  }

  var rawText = null;
  var rawTxtFile = null;
  var rawBinary = null;
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

  function formatBytes(n) {
    if (n >= 1073741824) return (n / 1073741824).toFixed(2) + " GB";
    if (n >= 1048576) return (n / 1048576).toFixed(1) + " MB";
    if (n >= 1024) return (n / 1024).toFixed(1) + " KB";
    return n + " B";
  }

  function setLoadLabel(msg) {
    const el = $("loadLabel");
    if (msg) {
      el.textContent = msg;
      el.hidden = false;
    } else {
      el.hidden = true;
      el.textContent = "";
    }
  }

  function setProgress(f) {
    const p = $("progress");
    const b = p.querySelector(".bar");
    if (f <= 0 || f >= 1) {
      if (f >= 1) {
        p.hidden = true;
        b.style.width = "0%";
        setLoadLabel(null);
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

  function axisLabel(key) {
    if (key.indexOf("Meta.") === 0) return "Meta";
    const m = key.match(/^(Arm\d+|Gripper\d+)/);
    if (m) return m[1];
    const i = key.lastIndexOf(".");
    return i > 0 ? key.slice(0, i) : key;
  }

  function groupLabel(key) {
    if (key.indexOf("Meta.") === 0) return "Meta";
    const i = key.lastIndexOf(".");
    return i > 0 ? key.slice(0, i) : key;
  }

  function syncSelectAllCheckbox(cb, keys) {
    let n = 0;
    for (let i = 0; i < keys.length; i++) {
      if (selected.has(keys[i])) n++;
    }
    cb.checked = keys.length > 0 && n === keys.length;
    cb.indeterminate = n > 0 && n < keys.length;
  }

  function setKeysSelected(keys, on) {
    for (let i = 0; i < keys.length; i++) {
      if (on) selected.add(keys[i]);
      else selected.delete(keys[i]);
    }
    saveSelectedKeys();
    redrawPlot();
    $("selCount").textContent = "已选 " + selected.size + " 条曲线";
  }

  function bindFieldCheckbox(cb, key, groupKeys, groupCb) {
    cb.checked = selected.has(key);
    cb.addEventListener("change", function () {
      if (cb.checked) selected.add(key);
      else selected.delete(key);
      saveSelectedKeys();
      if (groupCb) syncSelectAllCheckbox(groupCb, groupKeys);
      redrawPlot();
      $("selCount").textContent = "已选 " + selected.size + " 条曲线";
    });
  }

  function appendSelectAllSummary(det, title, keys, titleHint) {
    const sum = document.createElement("summary");
    sum.className = "tree-axis-summary";
    const lab = document.createElement("label");
    lab.className = "tree-select-all";
    const gcb = document.createElement("input");
    gcb.type = "checkbox";
    gcb.title = titleHint || "全选";
    syncSelectAllCheckbox(gcb, keys);
    lab.addEventListener("click", function (ev) {
      ev.stopPropagation();
    });
    gcb.addEventListener("change", function () {
      setKeysSelected(keys, gcb.checked);
      renderTree();
    });
    lab.appendChild(gcb);
    lab.appendChild(document.createTextNode(title));
    sum.appendChild(lab);
    det.appendChild(sum);
    return gcb;
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
      tree.textContent = "请先加载 TXT 或 BIN";
      $("selCount").textContent = "";
      return;
    }
    const vis = visibleMeta();
    const axes = new Map();
    for (let i = 0; i < vis.length; i++) {
      const m = vis[i];
      const ax = axisLabel(m.key);
      if (!axes.has(ax)) axes.set(ax, new Map());
      const sub = axes.get(ax);
      const g = groupLabel(m.key);
      if (!sub.has(g)) sub.set(g, []);
      sub.get(g).push(m);
    }
    const axisKeys = Array.from(axes.keys()).sort();
    for (let ai = 0; ai < axisKeys.length; ai++) {
      const axk = axisKeys[ai];
      const subGroups = axes.get(axk);
      const axisFieldKeys = [];
      subGroups.forEach(function (items) {
        for (let j = 0; j < items.length; j++) axisFieldKeys.push(items[j].key);
      });

      const axDet = document.createElement("details");
      axDet.open = true;
      const axisCb = appendSelectAllSummary(axDet, axk, axisFieldKeys, "全选本轴");

      const subKeys = Array.from(subGroups.keys()).sort();
      const isSingleGroup = subKeys.length === 1 && subKeys[0] === axk;
      if (isSingleGroup) {
        const items = subGroups.get(subKeys[0]);
        const ul = document.createElement("ul");
        for (let i = 0; i < items.length; i++) {
          const m = items[i];
          const li = document.createElement("li");
          const lab = document.createElement("label");
          const cb = document.createElement("input");
          cb.type = "checkbox";
          bindFieldCheckbox(cb, m.key, axisFieldKeys, axisCb);
          lab.appendChild(cb);
          const sp = document.createElement("span");
          sp.textContent = fieldTail(m.key);
          lab.appendChild(sp);
          li.appendChild(lab);
          ul.appendChild(li);
        }
        axDet.appendChild(ul);
      } else {
        for (let gi = 0; gi < subKeys.length; gi++) {
          const gk = subKeys[gi];
          const items = subGroups.get(gk);
          const groupFieldKeys = items.map(function (m) {
            return m.key;
          });
          const det = document.createElement("details");
          det.open = true;
          const groupCb = appendSelectAllSummary(det, gk, groupFieldKeys, "全选本组");
          const ul = document.createElement("ul");
          for (let i = 0; i < items.length; i++) {
            const m = items[i];
            const li = document.createElement("li");
            const lab = document.createElement("label");
            const cb = document.createElement("input");
            cb.type = "checkbox";
            bindFieldCheckbox(cb, m.key, groupFieldKeys, groupCb);
            cb.addEventListener("change", function () {
              syncSelectAllCheckbox(axisCb, axisFieldKeys);
            });
            lab.appendChild(cb);
            const sp = document.createElement("span");
            sp.textContent = fieldTail(m.key);
            lab.appendChild(sp);
            li.appendChild(lab);
            ul.appendChild(li);
          }
          det.appendChild(ul);
          axDet.appendChild(det);
        }
      }
      tree.appendChild(axDet);
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
          : parsed.timeSource === "decimated_index"
            ? "source row index (decimated)"
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

  function formatParseMetaNote(m) {
    if (!m) return "";
    const parts = [];
    if (m.estimatedSourceRows) {
      parts.push("源约 " + m.estimatedSourceRows.toLocaleString() + " 行");
    }
    if (m.decimateStride > 1) {
      parts.push("每 " + m.decimateStride + " 行取 1");
      if (m.autoDecimateApplied) parts.push("自动");
    }
    if (m.skipRows > 0) parts.push("跳过前 " + m.skipRows.toLocaleString());
    if (m.truncated) parts.push("已截断");
    return parts.length ? " · " + parts.join(" · ") : "";
  }

  function parseNow() {
    if (!rawText && !rawBinary && !rawTxtFile) return;
    setErr(null);
    var s;
    var parseOpts;
    try {
      s = readStructureFromForm();
      parseOpts = readParseOptionsFromForm();
      saveParseOptionsForm(parseOpts);
      if (loadedColumns && loadedColumns.length) s.columns = loadedColumns;
      else if (rawText) {
        const colCount = sniffTxtColumnCount(rawText);
        if (colCount > 0) {
          const plan = resolveTxtParsePlan(colCount, s);
          setActiveSchema(plan.schemaId);
          s = plan.structure;
        }
      }
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
      " · schema=" +
      schemaLabel() +
      " · 流式";
    setProgress(0.01);
    if (rawTxtFile) {
      setLoadLabel("流式解析 " + fileName + "（" + formatBytes(rawTxtFile.size) + "）…");
    }
    var parsePromise = rawBinary
      ? parseLogBinary(rawBinary, s, function (f) {
          setProgress(f);
        })
      : rawTxtFile
        ? parseLogTextFromFile(rawTxtFile, s, parseOpts, function (f) {
            setProgress(f);
          })
        : parseLogText(rawText, s, parseOpts, function (f) {
            setProgress(f);
          });
    parsePromise
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
        } else if (p.timeSource === "decimated_index") {
          timeNote = " · 时间轴为源行号（降采样）";
        } else {
          timeNote = " · 时间轴为行号";
        }
        var binNote = "";
        if (p.binaryHeader) {
          binNote =
            " · CTLG v" +
            p.binaryHeader.version +
            " · record_size=" +
            p.binaryHeader.record_size;
        }
        $("stats").textContent =
          p.rowCount.toLocaleString() +
          " 行 · " +
          p.seriesKeys.length +
          " 列" +
          binNote +
          timeNote +
          formatParseMetaNote(p.parseMeta) +
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

  function loadDataFile(f) {
    if (!f) return;
    fileName = f.name.replace(/\.[^.]+$/, "") || "chart";
    const isBin = /\.bin$/i.test(f.name);
    if (isBin) {
      const r = new FileReader();
      r.onload = function () {
        try {
          rawBinary = r.result;
          rawText = null;
          rawTxtFile = null;
          useSampleTimeAxis = false;
          const header = TelemetryBinary.readHeader(rawBinary);
          const structure = TelemetryBinary.structureFromHeader(header);
          const flatNames = TelemetryBinary.columnNamesV1(header);
          loadedColumns = TelemetryBinary.flatNamesToViewerKeys(flatNames);
          applyStructureToForm(structure);
          saveStructureForm(structure);
          setErr(null);
          parseNow();
        } catch (e) {
          rawBinary = null;
          setErr("BIN: " + e.message);
        }
      };
      r.readAsArrayBuffer(f);
      return;
    }
    rawTxtFile = f;
    rawText = null;
    rawBinary = null;
    useSampleTimeAxis = false;
    loadedColumns = null;
    setErr(null);
    setProgress(0.01);
    setLoadLabel("加载 " + f.name + "（" + formatBytes(f.size) + "）…");
    schemasReady
      .then(function () {
        return sniffTxtFile(f);
      })
      .then(function (colCount) {
        const plan = resolveTxtParsePlan(colCount, readStructureFromForm());
        setActiveSchema(plan.schemaId);
        applyStructureToForm(plan.structure);
        saveStructureForm(plan.structure);
        parseNow();
      })
      .catch(function (e) {
        rawTxtFile = null;
        setProgress(1);
        setErr(e.message || String(e));
      });
  }

  function loadJsonFile(f) {
    if (!f) return;
    const r = new FileReader();
    r.onload = function () {
      try {
        const j = JSON.parse(String(r.result));
        const root = j.structure && typeof j.structure === "object" ? j.structure : j;
        const s = normalizeStructureJson(root);
        applyStructureToForm(s);
        if (!rawBinary) loadedColumns = s.columns || null;
        saveStructureForm(s);
        setErr(null);
      } catch (e) {
        setErr("JSON: " + e.message);
      }
      if (rawBinary || rawText || rawTxtFile) parseNow();
    };
    r.readAsText(f);
  }

  function hasFileDrag(ev) {
    const dt = ev.dataTransfer;
    if (!dt || !dt.types) return false;
    return Array.prototype.indexOf.call(dt.types, "Files") >= 0;
  }

  function preventDragDefaults(ev) {
    ev.preventDefault();
    ev.stopPropagation();
  }

  var dragDepth = 0;

  function showDropOverlay() {
    const el = $("dropOverlay");
    if (el) el.hidden = false;
  }

  function hideDropOverlay() {
    dragDepth = 0;
    const el = $("dropOverlay");
    if (el) el.hidden = true;
  }

  document.addEventListener("dragenter", function (ev) {
    if (!hasFileDrag(ev)) return;
    preventDragDefaults(ev);
    dragDepth++;
    showDropOverlay();
  });

  document.addEventListener("dragover", function (ev) {
    if (!hasFileDrag(ev)) return;
    preventDragDefaults(ev);
  });

  document.addEventListener("dragleave", function (ev) {
    if (!hasFileDrag(ev)) return;
    preventDragDefaults(ev);
    dragDepth--;
    if (dragDepth <= 0) hideDropOverlay();
  });

  document.addEventListener("drop", function (ev) {
    if (!hasFileDrag(ev)) return;
    preventDragDefaults(ev);
    hideDropOverlay();
    const files = ev.dataTransfer && ev.dataTransfer.files;
    if (!files || !files.length) return;
    const f = files[0];
    const name = f.name.toLowerCase();
    if (name.endsWith(".json")) {
      loadJsonFile(f);
    } else if (name.endsWith(".txt") || name.endsWith(".bin")) {
      loadDataFile(f);
    } else {
      setErr("不支持的文件类型，请拖入 .txt / .bin / .json");
    }
  });

  $("dataFile").addEventListener("change", function (ev) {
    loadDataFile(ev.target.files[0]);
    ev.target.value = "";
  });

  $("jsonFile").addEventListener("change", function (ev) {
    loadJsonFile(ev.target.files[0]);
    ev.target.value = "";
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

  $("selVel").addEventListener("click", function () {
    if (!parsed) return;
    selected.clear();
    visibleMeta().forEach(function (m) {
      const k = m.key;
      if (
        k.indexOf(".joint_velocity") !== -1 ||
        k.indexOf(".motor_velocity") !== -1 ||
        k.indexOf(".filtered_joint_vel") !== -1
      ) {
        selected.add(k);
      }
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
  loadParseOptionsForm();
  loadSelectedKeys();
  refreshColInfo();

  function loadSchemaJson(url, assign) {
    return fetch(url)
      .then(function (r) {
        if (!r.ok) throw new Error("HTTP " + r.status);
        return r.json();
      })
      .then(function (j) {
        if (j && Array.isArray(j.meta) && Array.isArray(j.per_arm)) assign(j);
      })
      .catch(function () {});
  }

  var schemasReady = Promise.all([
    loadSchemaJson("log_schema.json", function (j) {
      logSchema = j;
    }),
    loadSchemaJson("log_schema_rt_control.json", function (j) {
      rtControlSchema = j;
    }),
  ]).then(function () {
    refreshColInfo();
  });
})();

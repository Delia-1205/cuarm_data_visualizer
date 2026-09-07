/* global TelemetryBinary */
(function (global) {
  "use strict";

  var MAGIC = 0x474c5443; // "CTLG" little-endian
  var HEADER_SIZE = 21;
  var MAX_ARM_SIZE = 2;
  var MAX_ARM_JOINT_SIZE = 7;
  var MAX_GRIPPER_SIZE = 2;
  var MAX_GRIPPER_JOINT_SIZE = 6;
  var RECORD_SIZE_V1 = 2120;
  var TOOL_POSE_LABELS = ["x", "y", "z", "qw", "qx", "qy", "qz"];
  var TASK_LABELS = ["x", "y", "z", "rx", "ry", "rz"];
  var META_FIELDS = [
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
  ];
  var META_SET = new Set(META_FIELDS);

  function readU64(dv, o) {
    var lo = dv.getUint32(o, true);
    var hi = dv.getUint32(o + 4, true);
    return hi * 0x100000000 + lo;
  }

  function readI64(dv, o) {
    var lo = dv.getUint32(o, true);
    var hi = dv.getInt32(o + 4, true);
    return hi * 0x100000000 + lo;
  }

  function readMatrixF32(dv, oRef, rows, cols) {
    var m = [];
    for (var r = 0; r < rows; r++) {
      m[r] = [];
      for (var c = 0; c < cols; c++) {
        m[r][c] = dv.getFloat32(oRef.o, true);
        oRef.o += 4;
      }
    }
    return m;
  }

  function readMatrixF64(dv, oRef, rows, cols) {
    var m = [];
    for (var r = 0; r < rows; r++) {
      m[r] = [];
      for (var c = 0; c < cols; c++) {
        m[r][c] = dv.getFloat64(oRef.o, true);
        oRef.o += 8;
      }
    }
    return m;
  }

  function readHeader(buffer) {
    if (buffer.byteLength < HEADER_SIZE) {
      throw new Error("文件过小，无法读取 CTLG 头");
    }
    var dv = new DataView(buffer);
    if (dv.getUint32(0, true) !== MAGIC) {
      throw new Error("无效的 CTLG 遥测文件（magic 不匹配）");
    }
    return {
      version: dv.getUint16(4, true),
      arm_size: dv.getUint16(6, true),
      arm_joint_size: [dv.getUint8(8), dv.getUint8(9)],
      gripper_size: dv.getUint8(10),
      gripper_joint_size: [dv.getUint8(11), dv.getUint8(12)],
      record_size: dv.getUint32(13, true),
    };
  }

  function structureFromHeader(header) {
    var armSize = header.arm_size;
    var jointSize = header.arm_joint_size.slice(0, armSize);
    while (jointSize.length < armSize) jointSize.push(0);
    var gripperSize = header.gripper_size;
    var gripperJointSize = header.gripper_joint_size
      .slice(0, gripperSize)
      .filter(function (n) {
        return n > 0;
      });
    return {
      ArmSize: armSize,
      JointSize: jointSize,
      GripperSize: gripperJointSize.length,
      GripperJointSize: gripperJointSize,
    };
  }

  function columnNamesV1(header) {
    var names = META_FIELDS.slice();
    var armCount = header ? header.arm_size : MAX_ARM_SIZE;
    var griCount = header ? header.gripper_size : MAX_GRIPPER_SIZE;
    for (var armI = 0; armI < armCount; armI++) {
      var jointCount = header ? header.arm_joint_size[armI] : MAX_ARM_JOINT_SIZE;
      for (var j = 0; j < jointCount; j++) {
        names.push(
          "arm" + armI + "_j" + j + "_motor_target",
          "arm" + armI + "_j" + j + "_joint_position",
          "arm" + armI + "_j" + j + "_joint_velocity",
          "arm" + armI + "_j" + j + "_joint_acceleration",
          "arm" + armI + "_j" + j + "_joint_torque",
          "arm" + armI + "_j" + j + "_motor_position",
          "arm" + armI + "_j" + j + "_motor_velocity",
          "arm" + armI + "_j" + j + "_motor_torque"
        );
      }
      for (var tp = 0; tp < 7; tp++) {
        names.push("arm" + armI + "_tool_pose" + tp);
      }
      for (var j2 = 0; j2 < jointCount; j2++) {
        names.push(
          "arm" + armI + "_j" + j2 + "_target_before_interpolation",
          "arm" + armI + "_j" + j2 + "_sliced_tar",
          "arm" + armI + "_j" + j2 + "_pid_cmd",
          "arm" + armI + "_j" + j2 + "_guard_pos_cmd",
          "arm" + armI + "_j" + j2 + "_sat_cmd",
          "arm" + armI + "_j" + j2 + "_jump_cmd"
        );
      }
      for (var tj = 0; tj < 6; tj++) {
        names.push(
          "arm" + armI + "_task_target_before_interpolation" + tj,
          "arm" + armI + "_task_sliced_tar" + tj,
          "arm" + armI + "_task_pose" + tj
        );
      }
      for (var j3 = 0; j3 < jointCount; j3++) {
        names.push(
          "arm" + armI + "_j" + j3 + "_joint_gravity",
          "arm" + armI + "_j" + j3 + "_friction",
          "arm" + armI + "_j" + j3 + "_gravity_kd_effect",
          "arm" + armI + "_j" + j3 + "_filtered_joint_vel",
          "arm" + armI + "_j" + j3 + "_arm_diagnostic_flags"
        );
      }
    }
    for (var griI = 0; griI < griCount; griI++) {
      var gjCount = header ? header.gripper_joint_size[griI] : MAX_GRIPPER_JOINT_SIZE;
      for (var gj = 0; gj < gjCount; gj++) {
        names.push(
          "gripper" + griI + "_j" + gj + "_target",
          "gripper" + griI + "_j" + gj + "_position",
          "gripper" + griI + "_j" + gj + "_diagnostic_flags"
        );
      }
    }
    return names;
  }

  function flatNameToViewerKey(name) {
    if (META_SET.has(name)) return "Meta." + name;

    var m = name.match(/^arm(\d+)_j(\d+)_(.+)$/);
    if (m) return "Arm" + m[1] + ".J" + (Number(m[2]) + 1) + "." + m[3];

    m = name.match(/^arm(\d+)_tool_pose(\d+)$/);
    if (m) {
      var ti = Number(m[2]);
      if (ti < TOOL_POSE_LABELS.length) {
        return "Arm" + m[1] + ".tool_pose_" + TOOL_POSE_LABELS[ti];
      }
    }

    m = name.match(/^arm(\d+)_(task_target_before_interpolation|task_sliced_tar|task_pose)(\d+)$/);
    if (m) {
      var li = Number(m[3]);
      if (li < TASK_LABELS.length) {
        var taskField = m[2].replace(/^task_/, "");
        return "Arm" + m[1] + ".task_" + taskField + "_" + TASK_LABELS[li];
      }
    }

    m = name.match(/^gripper(\d+)_j(\d+)_(target|position|diagnostic_flags)$/);
    if (m) {
      var field =
        m[3] === "target"
          ? "gripper_target"
          : m[3] === "position"
            ? "gripper_position"
            : "gripper_diagnostic_flags";
      return "Gripper" + (Number(m[1]) + 1) + ".J" + (Number(m[2]) + 1) + "." + field;
    }

    return name;
  }

  function flatNamesToViewerKeys(flatNames) {
    return flatNames.map(flatNameToViewerKey);
  }

  function parseSaveDataV1(dv, baseOffset) {
    var o = baseOffset;
    var simulation = dv.getInt32(o, true);
    o += 4;
    var target_type = dv.getInt32(o, true);
    o += 4;
    var actuator_mode = dv.getInt32(o, true);
    o += 4;
    var control_strategy = dv.getInt32(o, true);
    o += 4;
    var branch_state = dv.getInt32(o, true);
    o += 4;
    var system_state = dv.getInt32(o, true);
    o += 4;
    var plan_result = dv.getInt32(o, true);
    o += 4;
    o += 4; // align before uint64
    var system_diagnostic_flags = readU64(dv, o);
    o += 8;

    var arm_diagnostic_flags = [];
    for (var ai = 0; ai < MAX_ARM_SIZE; ai++) {
      arm_diagnostic_flags[ai] = [];
      for (var aj = 0; aj < MAX_ARM_JOINT_SIZE; aj++) {
        arm_diagnostic_flags[ai][aj] = readU64(dv, o);
        o += 8;
      }
    }

    var gripper_diagnostic_flags = [];
    for (var gi = 0; gi < MAX_GRIPPER_SIZE; gi++) {
      gripper_diagnostic_flags[gi] = [];
      for (var gj = 0; gj < MAX_GRIPPER_JOINT_SIZE; gj++) {
        gripper_diagnostic_flags[gi][gj] = readU64(dv, o);
        o += 8;
      }
    }

    var button_state = dv.getInt32(o, true);
    o += 4;
    o += 4; // align before int64 costs

    var communication_cost = readI64(dv, o);
    o += 8;
    var hardware_communication_cost = readI64(dv, o);
    o += 8;
    var running_cost = readI64(dv, o);
    o += 8;
    var robotics_total_cost = readI64(dv, o);
    o += 8;
    var motor_communication_cost = readI64(dv, o);
    o += 8;
    var gripper_communication_cost = readI64(dv, o);
    o += 8;
    var button_communication_cost = readI64(dv, o);
    o += 8;

    var oRef = { o: o };
    var gripper_target = readMatrixF32(dv, oRef, MAX_ARM_SIZE, MAX_GRIPPER_JOINT_SIZE);
    var gripper_position = readMatrixF32(dv, oRef, MAX_ARM_SIZE, MAX_GRIPPER_JOINT_SIZE);
    var joint_gravity = readMatrixF32(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var filtered_joint_vel = readMatrixF32(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var friction = readMatrixF32(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var gravity_kd_effect = readMatrixF32(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var motor_target = readMatrixF64(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var joint_position = readMatrixF64(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var joint_velocity = readMatrixF64(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var joint_acceleration = readMatrixF64(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var joint_torque = readMatrixF64(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var motor_velocity = readMatrixF64(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var motor_torque = readMatrixF64(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var motor_position = readMatrixF64(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var tool_pose = readMatrixF64(dv, oRef, MAX_ARM_SIZE, 7);
    var target_before_interpolation = readMatrixF64(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var sliced_tar = readMatrixF64(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var task_target_before_interpolation = readMatrixF32(dv, oRef, MAX_ARM_SIZE, 6);
    var task_sliced_tar = readMatrixF32(dv, oRef, MAX_ARM_SIZE, 6);
    var task_pose = readMatrixF32(dv, oRef, MAX_ARM_SIZE, 6);
    var pid_cmd = readMatrixF32(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var guard_pos_cmd = readMatrixF32(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var sat_cmd = readMatrixF32(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);
    var jump_cmd = readMatrixF32(dv, oRef, MAX_ARM_SIZE, MAX_ARM_JOINT_SIZE);

    return {
      simulation: simulation,
      target_type: target_type,
      actuator_mode: actuator_mode,
      control_strategy: control_strategy,
      branch_state: branch_state,
      system_state: system_state,
      plan_result: plan_result,
      system_diagnostic_flags: system_diagnostic_flags,
      arm_diagnostic_flags: arm_diagnostic_flags,
      gripper_diagnostic_flags: gripper_diagnostic_flags,
      button_state: button_state,
      communication_cost: communication_cost,
      hardware_communication_cost: hardware_communication_cost,
      running_cost: running_cost,
      robotics_total_cost: robotics_total_cost,
      motor_communication_cost: motor_communication_cost,
      gripper_communication_cost: gripper_communication_cost,
      button_communication_cost: button_communication_cost,
      gripper_target: gripper_target,
      gripper_position: gripper_position,
      joint_gravity: joint_gravity,
      filtered_joint_vel: filtered_joint_vel,
      friction: friction,
      gravity_kd_effect: gravity_kd_effect,
      motor_target: motor_target,
      joint_position: joint_position,
      joint_velocity: joint_velocity,
      joint_acceleration: joint_acceleration,
      joint_torque: joint_torque,
      motor_velocity: motor_velocity,
      motor_torque: motor_torque,
      motor_position: motor_position,
      tool_pose: tool_pose,
      target_before_interpolation: target_before_interpolation,
      sliced_tar: sliced_tar,
      task_target_before_interpolation: task_target_before_interpolation,
      task_sliced_tar: task_sliced_tar,
      task_pose: task_pose,
      pid_cmd: pid_cmd,
      guard_pos_cmd: guard_pos_cmd,
      sat_cmd: sat_cmd,
      jump_cmd: jump_cmd,
    };
  }

  function flattenRecordV1(record, header) {
    var row = [
      record.running_cost,
      record.robotics_total_cost,
      record.hardware_communication_cost,
      record.communication_cost,
      record.motor_communication_cost,
      record.gripper_communication_cost,
      record.button_communication_cost,
      record.actuator_mode,
      record.control_strategy,
      record.branch_state,
      record.system_state,
      record.plan_result,
      record.system_diagnostic_flags,
      record.button_state,
      record.simulation,
      record.target_type,
    ];

    for (var armI = 0; armI < header.arm_size; armI++) {
      var jointCount = header.arm_joint_size[armI];
      for (var j = 0; j < jointCount; j++) {
        row.push(
          record.motor_target[armI][j],
          record.joint_position[armI][j],
          record.joint_velocity[armI][j],
          record.joint_acceleration[armI][j],
          record.joint_torque[armI][j],
          record.motor_position[armI][j],
          record.motor_velocity[armI][j],
          record.motor_torque[armI][j]
        );
      }
      for (var tp = 0; tp < 7; tp++) {
        row.push(record.tool_pose[armI][tp]);
      }
      for (var j2 = 0; j2 < jointCount; j2++) {
        row.push(
          record.target_before_interpolation[armI][j2],
          record.sliced_tar[armI][j2],
          record.pid_cmd[armI][j2],
          record.guard_pos_cmd[armI][j2],
          record.sat_cmd[armI][j2],
          record.jump_cmd[armI][j2]
        );
      }
      for (var tj = 0; tj < 6; tj++) {
        row.push(
          record.task_target_before_interpolation[armI][tj],
          record.task_sliced_tar[armI][tj],
          record.task_pose[armI][tj]
        );
      }
      for (var j3 = 0; j3 < jointCount; j3++) {
        row.push(
          record.joint_gravity[armI][j3],
          record.friction[armI][j3],
          record.gravity_kd_effect[armI][j3],
          record.filtered_joint_vel[armI][j3],
          record.arm_diagnostic_flags[armI][j3]
        );
      }
    }

    for (var griI = 0; griI < header.gripper_size; griI++) {
      var gjCount = header.gripper_joint_size[griI];
      for (var gj = 0; gj < gjCount; gj++) {
        row.push(
          record.gripper_target[griI][gj],
          record.gripper_position[griI][gj],
          record.gripper_diagnostic_flags[griI][gj]
        );
      }
    }
    return row;
  }

  function countRecords(buffer, header) {
    if (header.version !== 1) {
      throw new Error("不支持的遥测版本 " + header.version + "（当前仅支持 version=1）");
    }
    if (header.record_size !== RECORD_SIZE_V1) {
      throw new Error(
        "record_size 不匹配：文件=" + header.record_size + "，解析器=" + RECORD_SIZE_V1
      );
    }
    var payload = buffer.byteLength - HEADER_SIZE;
    if (payload < 0 || payload % header.record_size !== 0) {
      throw new Error("二进制 payload 长度与 record_size 不对齐");
    }
    return payload / header.record_size;
  }

  /**
   * Parse CTLG .bin into row-major numeric table.
   * Returns { header, flatColumnNames, viewerColumnNames, rows }.
   */
  function parseBinaryTable(buffer, onProgress) {
    return new Promise(function (resolve, reject) {
      try {
        var header = readHeader(buffer);
        var rowCount = countRecords(buffer, header);
        if (rowCount === 0) {
          reject(new Error("二进制文件无记录（仅有 CTLG 头）"));
          return;
        }
        var flatColumnNames = columnNamesV1(header);
        var viewerColumnNames = flatNamesToViewerKeys(flatColumnNames);
        var nCols = flatColumnNames.length;
        var rows = new Array(rowCount);
        var dv = new DataView(buffer);
        var offset = HEADER_SIZE;
        var r = 0;
        var chunk = 500;

        function step() {
          try {
            var end = Math.min(r + chunk, rowCount);
            for (; r < end; r++) {
              var record = parseSaveDataV1(dv, offset);
              offset += header.record_size;
              rows[r] = flattenRecordV1(record, header);
            }
            if (onProgress) onProgress(rowCount ? r / rowCount : 1);
            if (r < rowCount) {
              setTimeout(step, 0);
            } else {
              resolve({
                header: header,
                flatColumnNames: flatColumnNames,
                viewerColumnNames: viewerColumnNames,
                rows: rows,
                rowCount: rowCount,
                nCols: nCols,
              });
            }
          } catch (e) {
            reject(e);
          }
        }
        step();
      } catch (e) {
        reject(e);
      }
    });
  }

  global.TelemetryBinary = {
    readHeader: readHeader,
    structureFromHeader: structureFromHeader,
    columnNamesV1: columnNamesV1,
    flatNameToViewerKey: flatNameToViewerKey,
    flatNamesToViewerKeys: flatNamesToViewerKeys,
    parseBinaryTable: parseBinaryTable,
  };
})(typeof window !== "undefined" ? window : globalThis);

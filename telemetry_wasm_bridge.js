/**
 * Optional WASM telemetry parser bridge.
 * When telemetry/build-wasm output is copied here as telemetry.js + telemetry.wasm,
 * binary parsing uses the shared C++ core; otherwise falls back to telemetry_binary.js.
 */
(function (global) {
  "use strict";

  var wasmModulePromise = null;

  function wasmEnabled() {
    return typeof global.createTelemetryModule === "function";
  }

  function getWasmModule() {
    if (!wasmEnabled()) {
      return Promise.reject(new Error("telemetry WASM not loaded"));
    }
    if (!wasmModulePromise) {
      wasmModulePromise = global.createTelemetryModule();
    }
    return wasmModulePromise;
  }

  function convertWasmDataset(result) {
    var columns = [];
    var viewerKeys = [];
    for (var i = 0; i < result.columns.length; i++) {
      var c = result.columns[i];
      columns.push(c.flat_name);
      viewerKeys.push(c.display_key);
    }
    var rows = [];
    if (result.column_data && result.column_data.length) {
      var rowCount = result.row_count;
      for (var r = 0; r < rowCount; r++) {
        var row = [];
        for (var cidx = 0; cidx < result.column_data.length; cidx++) {
          row.push(result.column_data[cidx][r]);
        }
        rows.push(row);
      }
    }
    return {
      viewerColumnNames: viewerKeys,
      flatColumnNames: columns,
      rows: rows,
      rowCount: result.row_count,
      nCols: columns.length,
    };
  }

  function parseBinaryWithWasm(buffer, onProgress) {
    return getWasmModule().then(function (mod) {
      if (onProgress) onProgress(0.2);
      if (!mod.loadFile) {
        throw new Error("WASM module missing loadFile export");
      }
      // Emscripten FS path-based API requires mounting; use native JS fallback for File blobs.
      throw new Error("in-memory WASM parse not wired; use TelemetryBinary fallback");
    });
  }

  global.TelemetryWasmBridge = {
    wasmEnabled: wasmEnabled,
    parseBinaryWithWasm: parseBinaryWithWasm,
    convertWasmDataset: convertWasmDataset,
  };
})(typeof window !== "undefined" ? window : globalThis);

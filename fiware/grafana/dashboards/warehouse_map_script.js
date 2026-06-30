// Plotly.js script for the Warehouse Floor Map panel.
// This is a reference copy — the actual script is embedded
// in geofence_overview.json under panels[0].options.script.
// Edit this file for readability, then paste back into the JSON.

let traces = [];
let allX = [], allY = [];

// Trace 1: Wall segments (outer contour)
const wallData = data.series.find(s => s.refId === 'walls');
if (wallData && wallData.fields.length >= 4) {
  const x1 = wallData.fields.find(f => f.name === 'x1');
  const y1 = wallData.fields.find(f => f.name === 'y1');
  const x2 = wallData.fields.find(f => f.name === 'x2');
  const y2 = wallData.fields.find(f => f.name === 'y2');
  if (x1 && y1 && x2 && y2) {
    let wx = [], wy = [];
    for (let i = 0; i < x1.values.length; i++) {
      wx.push(x1.values[i], x2.values[i], null);
      wy.push(y1.values[i], y2.values[i], null);
      allX.push(x1.values[i], x2.values[i]);
      allY.push(y1.values[i], y2.values[i]);
    }
    traces.push({x: wx, y: wy, mode: 'lines', line: {color: '#aaaaaa', width: 3}, name: 'Walls'});
  }
}

// Trace 2: Inner polygons (contour outlines)
const innerData = data.series.find(s => s.refId === 'inner');
if (innerData && innerData.fields.length >= 3) {
  const px = innerData.fields.find(f => f.name === 'x');
  const py = innerData.fields.find(f => f.name === 'y');
  const pidx = innerData.fields.find(f => f.name === 'poly_idx');
  if (px && py && pidx) {
    let polyTraces = {};
    for (let i = 0; i < px.values.length; i++) {
      const idx = pidx.values[i];
      if (!polyTraces[idx]) polyTraces[idx] = {x: [], y: []};
      polyTraces[idx].x.push(px.values[i]);
      polyTraces[idx].y.push(py.values[i]);
      allX.push(px.values[i]);
      allY.push(py.values[i]);
    }
    let first = true;
    for (const idx of Object.keys(polyTraces)) {
      const p = polyTraces[idx];
      traces.push({x: p.x, y: p.y, mode: 'lines', line: {color: '#ff8800', width: 2}, fill: 'toself', fillcolor: 'rgba(255,136,0,0.15)', name: first ? 'Inner contour' : '', showlegend: first});
      first = false;
    }
  }
}

// Trace 3: Map obstacles (pallets — always visible, green filled rectangles)
const mapObsData = data.series.find(s => s.refId === 'map_obs_rects');
if (mapObsData && mapObsData.fields.length >= 4) {
  const rx0 = mapObsData.fields.find(f => f.name === 'x_min');
  const ry0 = mapObsData.fields.find(f => f.name === 'y_min');
  const rx1 = mapObsData.fields.find(f => f.name === 'x_max');
  const ry1 = mapObsData.fields.find(f => f.name === 'y_max');
  if (rx0 && ry0 && rx1 && ry1) {
    let first = true;
    for (let i = 0; i < rx0.values.length; i++) {
      const xA = rx0.values[i], yA = ry0.values[i];
      const xB = rx1.values[i], yB = ry1.values[i];
      traces.push({x: [xA,xB,xB,xA,xA], y: [yA,yA,yB,yB,yA], mode: 'lines', fill: 'toself', fillcolor: 'rgba(68,204,68,0.25)', line: {color: '#44cc44', width: 1}, name: first ? 'Pallets' : '', showlegend: first});
      first = false;
    }
  }
}

// Trace 4: Matched LiDAR segments (blue lines)
const matchedData = data.series.find(s => s.refId === 'matched');
if (matchedData && matchedData.fields.length >= 4) {
  const mx1 = matchedData.fields.find(f => f.name === 'x1');
  const my1 = matchedData.fields.find(f => f.name === 'y1');
  const mx2 = matchedData.fields.find(f => f.name === 'x2');
  const my2 = matchedData.fields.find(f => f.name === 'y2');
  if (mx1 && my1 && mx2 && my2) {
    let lx = [], ly = [];
    for (let i = 0; i < mx1.values.length; i++) {
      lx.push(mx1.values[i], mx2.values[i], null);
      ly.push(my1.values[i], my2.values[i], null);
    }
    traces.push({x: lx, y: ly, mode: 'lines', line: {color: '#4488ff', width: 2}, name: 'Matched (LiDAR)'});
  }
}

// Trace 5: Robot position trail
const robotData = data.series.find(s => s.refId === 'robot');
if (robotData && robotData.fields.length >= 2) {
  const rx = robotData.fields.find(f => f.name === 'x');
  const ry = robotData.fields.find(f => f.name === 'y');
  if (rx && ry && rx.values.length > 0) {
    traces.push({x: rx.values, y: ry.values, mode: 'lines+markers', marker: {size: 4, color: '#4488ff'}, line: {color: '#4488ff44', width: 1}, name: 'Robot trail'});
    const last = rx.values.length - 1;
    traces.push({x: [rx.values[last]], y: [ry.values[last]], mode: 'markers', marker: {size: 14, color: '#4488ff', symbol: 'diamond'}, name: 'Robot (now)'});
  }
}

// Trace 7: Unmatched obstacles (red lines)
const unmatchedData = data.series.find(s => s.refId === 'unmatched');
if (unmatchedData && unmatchedData.fields.length >= 4) {
  const ux1 = unmatchedData.fields.find(f => f.name === 'x1');
  const uy1 = unmatchedData.fields.find(f => f.name === 'y1');
  const ux2 = unmatchedData.fields.find(f => f.name === 'x2');
  const uy2 = unmatchedData.fields.find(f => f.name === 'y2');
  if (ux1 && uy1 && ux2 && uy2) {
    let lx = [], ly = [];
    for (let i = 0; i < ux1.values.length; i++) {
      lx.push(ux1.values[i], ux2.values[i], null);
      ly.push(uy1.values[i], uy2.values[i], null);
    }
    traces.push({x: lx, y: ly, mode: 'lines', line: {color: '#ff4444', width: 3}, name: 'Unmatched'});
  }
}

// Trace 6: Predicted heatmap (orange-red gradient circles)
const heatmapData = data.series.find(s => s.refId === 'heatmap');
if (heatmapData && heatmapData.fields.length >= 3) {
  const hx = heatmapData.fields.find(f => f.name === 'x');
  const hy = heatmapData.fields.find(f => f.name === 'y');
  const hv = heatmapData.fields.find(f => f.name === 'value');
  if (hx && hy && hv && hx.values.length > 0) {
    const sizes = hv.values.map(v => 6 + (v / 100.0) * 20);
    const colors = hv.values.map(v => `rgba(255, ${Math.max(0, 200 - v * 2)}, 0, ${0.3 + (v / 100.0) * 0.5})`);
    traces.push({x: hx.values, y: hy.values, mode: 'markers', marker: {size: sizes, color: colors}, name: 'Predicted', hovertext: hv.values.map(v => v + '%')});
  }
}

// Compute axis range with buffer
if (allX.length > 0) {
  const xMin = Math.min(...allX), xMax = Math.max(...allX);
  const yMin = Math.min(...allY), yMax = Math.max(...allY);
  const pad = Math.max((xMax - xMin), (yMax - yMin)) * 0.05 + 1;
  return {data: traces, layout: {xaxis: {range: [xMin - pad, xMax + pad], type: 'linear'}, yaxis: {range: [yMin - pad, yMax + pad], type: 'linear'}}};
}

return {data: traces, layout: {xaxis: {type: 'linear'}, yaxis: {type: 'linear'}}};
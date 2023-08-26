import Plotly from "plotly.js-dist-min";
import {numDerivOffline, numDerivOnline, inverseOverflow} from '../common.js';

function newChart(container, xs, ys, options) {
  if (xs.length !== ys.length) {
    throw new Error();
  }

  // cribbed from https://plotly.com/javascript/plotlyjs-events/#select-event
  const color = '#777';
  const colorLight = '#bbb';

  const chartDiv = document.createElement('div');
  Plotly.newPlot(chartDiv, [{
    type: 'scatter',
    mode: 'markers',
    x: xs,
    y: ys,
    name: 'Samples',
  }], {
    title: options.title || '',
    // sets the starting tool from the modebar
    dragmode: 'zoom',
    showlegend: false,
    hovermode: false,
    width: 600,
  }, {
    // 'zoom2d', 'select2d', 'resetScale2d' left
    modeBarButtonsToRemove: ['pan2d', 'lasso2d', 'zoomIn2d', 'zoomOut2d', 'autoScale2d'],
  });

  container.appendChild(chartDiv);
}

window.addEventListener("DOMContentLoaded", function() {
  const params = new URLSearchParams(window.location.search);
  const online = params.get('online') === 'true';

  fetch(`/tuning/test/${params.get('file')}.csv`)
    .then(res => res.text())
    .then(text => {
      const lines = text.split('\n');
      const data = lines
        .slice(1, lines.length - 1)
        .map(line => line.split(',').map(x => parseFloat(x)))
      
      const xs = data.map(xs => xs[0]);
      const vs = data.map(xs => xs[1]);
      const ts = data.map(xs => xs[2]);

      const container = document.getElementById('content');
      newChart(container, ts, vs, {title: 'Raw Velocity Over Time'});

      const tsNew = (online ? ts.slice(1) : ts.slice(1, ts.length - 1));
      const dxdts = (online ? numDerivOnline : numDerivOffline)(ts, xs);
      newChart(
        container, 
        tsNew,
        dxdts, 
        {title: `${online ? 'Online' : 'Offline'} Position Derivative Over Time`}
      );

      const vsNew = dxdts.map((est, i) => inverseOverflow(vs[i + 1], est));
      newChart(
        container, 
        tsNew,
        vsNew, 
        {title: 'Corrected Velocity Over Time'}
      );
    })
    .catch(console.log.bind(console));
});
import Plotly from 'plotly.js-dist-min';
import regression, { DataPoint } from 'regression';

// TODO: time-interpolate data

// https://en.wikipedia.org/wiki/Kahan_summation_algorithm#The_algorithm
function kahanSum(xs: number[]) {
  let sum = 0;
  let c = 0;

  for (let i = 0; i < xs.length; i++) {
    const y = xs[i] - c;
    const t = sum + y;
    c = (t - sum) - y;
    sum = t;
  }

  return sum;
}

// https://en.wikipedia.org/wiki/Simple_linear_regression#Simple_linear_regression_without_the_intercept_term_(single_regressor)
function fitLinearNoIntercept(xs: number[], ys: number[]) {
  return kahanSum(
    xs.map((x, i) => x * ys[i])
  ) / kahanSum(
    xs.map(x => x * x)
  );
}

function fitLinearWithScaling(xs: number[], ys: number[]) {
  const xOffset = xs.reduce((a, b) => a + b, 0) / xs.length;
  const yOffset = ys.reduce((a, b) => a + b, 0) / ys.length;

  const xScale = xs.reduce((acc, x) => Math.max(acc, Math.abs(x - xOffset)), 0);
  const yScale = ys.reduce((acc, y) => Math.max(acc, Math.abs(y - yOffset)), 0);

  const data: DataPoint[] = xs.map((x, i) => [(x - xOffset) / xScale, (ys[i] - yOffset) / yScale]);

  const result = regression.linear(data);
  const [m, b] = result.equation;

  return [m * yScale / xScale, b * yScale - m * xOffset * yScale / xScale + yOffset];
}

// no output for first pair
export function numDerivOnline(xs: number[], ys: number[]) {
  if (xs.length !== ys.length) {
    throw new Error(`${xs.length} !== ${ys.length}`);
  }

  return ys
    .slice(1)
    .map((y, i) => (y - ys[i]) / (xs[i + 1] - xs[i]));
}

// no output for first or last pair
export function numDerivOffline(xs: number[], ys: number[]) {
  return ys
    .slice(2)
    .map((y, i) => (y - ys[i]) / (xs[i + 2] - xs[i]));
}

const CPS_STEP = 0x10000;

export function inverseOverflow(input: number, estimate: number) {
  // convert to uint16
  let real = input & 0xffff;
  // initial, modulo-based correction: it can recover the remainder of 5 of the upper 16 bits
  // because the velocity is always a multiple of 20 cps due to Expansion Hub's 50ms measurement window
  real += ((real % 20) / 4) * CPS_STEP;
  // estimate-based correction: it finds the nearest multiple of 5 to correct the upper bits by
  real += Math.round((estimate - real) / (5 * CPS_STEP)) * 5 * CPS_STEP;
  return real;
}

// no output for first or last pair
function fixVels(ts: number[], xs: number[], vs: number[]) {
  if (ts.length !== xs.length || ts.length !== vs.length) {
    throw new Error();
  }

  return numDerivOffline(ts, xs).map((est, i) => inverseOverflow(vs[i + 1], est));
}

// see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
function fixAngVels(vs: number[]) {
  if (vs.length === 0) {
    return [];
  }

  let offset = 0;
  let lastV = vs[0];
  const vsFixed = [lastV];
  for (let i = 1; i < vs.length; i++) {
    if (Math.abs(vs[i] - lastV) > Math.PI) {
      offset -= Math.sign(vs[i] - lastV) * 2 * Math.PI;
    }
    vsFixed.push(offset + vs[i]);
    lastV = vs[i];
  }

  return vsFixed;
}

type RegressionOptions = {
  title: string;
  slope: string;
  intercept?: string;
};

// data comes in pairs
async function newLinearRegressionChart(container: HTMLElement, xs: number[], ys: number[], options: RegressionOptions, onChange?: (m: number, b: number) => void): Promise<(xs: number[], ys: number[]) => void> {
  if (xs.length !== ys.length) {
    throw new Error(`${xs.length} !== ${ys.length}`);
  }

  // cribbed from https://plotly.com/javascript/plotlyjs-events/#select-event
  const color = '#777';
  const colorLight = '#bbb';

  let mask = xs.map(() => true);

  function fit(xs: number[], ys: number[]) {
    return options.intercept === undefined ? [fitLinearNoIntercept(xs, ys), 0] : fitLinearWithScaling(xs, ys);
  }

  const [m, b] = fit(xs, ys);

  if (onChange) onChange(m, b);

  const minX = xs.reduce((a, b) => Math.min(a, b), 0);
  const maxX = xs.reduce((a, b) => Math.max(a, b), 0);

  const chartDiv = document.createElement('div');
  const width = Math.max(0, window.innerWidth - 50);
  // TODO: is it really worth awaiting here instead of using chartDiv directly?
  const plot = await Plotly.newPlot(chartDiv, [{
    type: 'scatter',
    mode: 'markers',
    x: xs,
    y: ys,
    name: 'Samples',
    // markers seem to respond to selection 
    marker: { color: mask.map(b => b ? color : colorLight), size: 5 },
  }, {
    type: 'scatter',
    mode: 'lines',
    x: [minX, maxX],
    y: [m * minX + b, m * maxX + b],
    name: 'Regression Line',
    line: { color: 'red' }
  }], {
    title: options.title || '',
    // sets the starting tool from the modebar
    dragmode: 'select',
    showlegend: false,
    hovermode: false,
    width,
    height: width * 9 / 16,
  }, {
    // 'select2d', 'zoom2d', 'pan2d', 'lasso2d', 'zoomIn2d', 'zoomOut2d', 'autoScale2d', 'resetScale2d' left
    modeBarButtonsToRemove: [],
  });

  const results = document.createElement('p');

  function setResultText(m: number, b: number) {
    results.innerText = `${options.slope || 'slope'}: ${m}, ${options.intercept || 'y-intercept'}: ${b}`;
  }
  setResultText(m, b);

  function updatePlot() {
    // TODO: this seems to work? maybe there aren't type definitions yet?
    Plotly.restyle(chartDiv, 'marker.color', [
      mask.map(b => b ? color : colorLight)
      // @ts-ignore
    ], [0]);

    const [m, b] = fit(
      xs.filter((_, i) => mask[i]),
      ys.filter((_, i) => mask[i]),
    );
    setResultText(m, b);
    if (onChange) onChange(m, b);

    const minX = xs.reduce((a, b) => Math.min(a, b));
    const maxX = xs.reduce((a, b) => Math.max(a, b));

    Plotly.restyle(chartDiv, {
      x: [[minX, maxX]],
      y: [[m * minX + b, m * maxX + b]],
    }, [1]);
  }

  let pendingSelection: Plotly.PlotSelectionEvent | null = null;

  plot.on('plotly_selected', function (eventData) {
    if (eventData === undefined) {
      return;
    }

    pendingSelection = eventData;
  });

  function applyPendingSelection(b: boolean) {
    if (pendingSelection === null) return false;

    for (const pt of pendingSelection.points) {
      mask[pt.pointIndex] = b;
    }

    // @ts-ignore
    Plotly.restyle(chartDiv, 'selectedpoints', [null], [0]);

    pendingSelection = null;

    return true;
  }

  const includeButton = document.createElement('button');
  includeButton.innerText = '[i]nclude';
  includeButton.addEventListener('click', () => {
    if (!applyPendingSelection(true)) return;
    updatePlot();
  });

  const excludeButton = document.createElement('button');
  excludeButton.innerText = '[e]xclude';
  excludeButton.addEventListener('click', () => {
    if (!applyPendingSelection(false)) return;
    updatePlot();
  });

  document.addEventListener('keydown', e => {
    if (e.key === 'i') {
      if (!applyPendingSelection(true)) return;
      updatePlot();
    } else if (e.key === 'e') {
      if (!applyPendingSelection(false)) return;
      updatePlot();
    }
  });

  while (container.firstChild) {
    container.removeChild(container.firstChild);
  }

  const buttons = document.createElement('div');
  buttons.appendChild(includeButton);
  buttons.appendChild(excludeButton);

  const bar = document.createElement('div');
  bar.setAttribute('class', 'bar');
  bar.appendChild(buttons);

  bar.appendChild(results);

  container.appendChild(bar);
  container.appendChild(chartDiv);

  return function (xsNew: number[], ysNew: number[]) {
    if (xsNew.length !== ysNew.length) {
      throw new Error(`${xsNew.length} !== ${ysNew.length}`);
    }

    xs = xsNew;
    ys = ysNew;
    mask = xs.map(() => true);

    Plotly.restyle(chartDiv, {
      x: [xs],
      y: [ys],
    }, [0]);

    updatePlot();
  };
}


type Signal = {
  times: number[];
  values: number[];
};

type DriveType = 'tank' | 'mecanum';

type AngularRampData = {
  type: DriveType;
  leftPowers: Signal[];
  rightPowers: Signal[];
  voltages: Signal;
  leftEncPositions: Signal[];
  rightEncPositions: Signal[];
  parEncPositions: Signal[];
  perpEncPositions: Signal[];
  leftEncVels: Signal[];
  rightEncVels: Signal[];
  parEncVels: Signal[];
  perpEncVels: Signal[];
  angVels: [Signal, Signal, Signal];
};

// We skip the last measurement because it may be corrupted. Perhaps an artifact of
// poisoned Lynx modules?

// TODO: should this be tied to particular IDs?
export async function loadDeadWheelAngularRampRegression(data: AngularRampData) {
  const [_, angVels] = data.angVels.reduce((acc: [number, number[]], vsArg) => {
    const vs = fixAngVels(vsArg.values.slice(0, -1)).map(v => Math.abs(v));
    const maxV = vs.reduce((acc, v) => Math.max(acc, v), 0);
    const [accMaxV, _] = acc;
    if (maxV >= accMaxV) {
      return [maxV, vs];
    }
    return acc;
  }, [0, []]);

  const deadWheelCharts = document.getElementById('deadWheelCharts')!;
  data.parEncVels.forEach((vs, i) => {
    const div = document.createElement('div');
    newLinearRegressionChart(div,
      angVels.slice(1, -1),
      fixVels(
        vs.times.slice(0, -1),
        data.parEncPositions[i].values.slice(0, -1),
        vs.values.slice(0, -1)
      ),
      { title: `Parallel Wheel ${i} Regression`, slope: 'y-position' });
    deadWheelCharts.appendChild(div);
  });
  data.perpEncVels.forEach((vs, i) => {
    const div = document.createElement('div');
    newLinearRegressionChart(div,
      angVels.slice(1, -1),
      fixVels(
        vs.times.slice(0, -1),
        data.perpEncPositions[i].values.slice(0, -1),
        vs.values.slice(0, -1)
      ),
      { title: `Perpendicular Wheel ${i} Regression`, slope: 'x-position' });
    deadWheelCharts.appendChild(div);
  });

  const setParams = await (async () => {
    const allPowers = [...data.leftPowers, ...data.rightPowers];
    const appliedVoltages = data.voltages.values.slice(0, -1).map((v, i) =>
      allPowers.reduce((acc, ps) => Math.max(acc, ps.values[i]), 0) * v);

    const setTrackWidthData = await newLinearRegressionChart(
      document.getElementById('trackWidthChart')!,
      [], [],
      { title: 'Track Width Regression', slope: 'track width' },
    );

    return (kV: number, kS: number) => setTrackWidthData(angVels, appliedVoltages.map((v) =>
      (v - kS) / kV * (data.type === 'mecanum' ? 2 : 1)));
  })();

  const kvInput = document.getElementById('kv')! as HTMLInputElement;
  const ksInput = document.getElementById('ks')! as HTMLInputElement;
  document.getElementById('update')!.addEventListener('click', () => {
    setParams(parseFloat(kvInput.value), parseFloat(ksInput.value));
  });

  setParams(parseFloat(kvInput.value), parseFloat(ksInput.value));
}

export async function loadDriveEncoderAngularRampRegression(data: AngularRampData) {
  const leftEncVels = data.leftEncVels.map((vs, i) =>
    fixVels(vs.times.slice(0, -1), data.leftEncPositions[i].values.slice(0, -1), vs.values.slice(0, -1)));
  const rightEncVels = data.rightEncVels.map((vs, i) =>
    fixVels(vs.times.slice(0, -1), data.rightEncPositions[i].values.slice(0, -1), vs.values.slice(0, -1)));

  await newLinearRegressionChart(
    document.getElementById('rampChart')!,
    [
      ...leftEncVels.flatMap(vs => vs.map(v => -v)),
      ...rightEncVels.flatMap(vs => vs),
    ],
    [
      ...data.leftPowers.flatMap(ps => {
        const psNew = ps.values.slice(0, -1).map((p, i) => -p * data.voltages.values[i]);
        return psNew.slice(1, -1);
      }),
      ...data.rightPowers.flatMap(ps => {
        const psNew = ps.values.slice(0, -1).map((p, i) => p * data.voltages.values[i]);
        return psNew.slice(1, -1);
      }),
    ],
    { title: 'Angular Ramp Regression', slope: 'kV', intercept: 'kS' }
  );

  const p = data.angVels.reduce<[number, number[]]>((acc, vsArg) => {
    const vs = fixAngVels(vsArg.values.slice(0, -1)).map(v => Math.abs(v));
    const maxV = vs.reduce((acc, v) => Math.max(acc, v), 0);
    const [accMaxV, _] = acc;
    if (maxV >= accMaxV) {
      return [maxV, vs];
    }
    return acc;
  }, [0, []]);
  const angVels = p[1].slice(1, -1);

  await newLinearRegressionChart(
    document.getElementById('trackWidthChart')!,
    angVels,
    angVels.map((_, i) =>
      (leftEncVels.reduce((acc, vs) => acc - vs[i], 0) / data.leftEncVels.length
        + rightEncVels.reduce((acc, vs) => acc + vs[i], 0) / data.rightEncVels.length)
      * (data.type === 'mecanum' ? 0.5 : 1)
    ),
    { title: 'Track Width Regression', slope: 'track width' },
  );
}

type ForwardRampData = {
  type: DriveType;
  powers: Signal[];
  voltages: Signal;
  forwardEncPositions: Signal[];
  forwardEncVels: Signal[];
};

export async function loadForwardRampRegression(data: ForwardRampData) {
  const forwardEncVels = data.forwardEncVels.flatMap((vs, i) =>
    fixVels(vs.times.slice(0, -1), data.forwardEncPositions[i].values.slice(0, -1), vs.values.slice(0, -1)));
  const appliedVoltages = data.forwardEncVels.flatMap(() => {
    const voltages = data.voltages.values.slice(0, -1).map((v, i) =>
      data.powers.reduce((acc, ps) => Math.max(acc, ps.values[i]), 0) * v);

    return voltages.slice(1, voltages.length - 1);
  });

  await newLinearRegressionChart(
    document.getElementById('rampChart')!,
    forwardEncVels, appliedVoltages,
    { title: 'Forward Ramp Regression', slope: 'kV', intercept: 'kS' },
  );
}

type LateralRampData = {
  type: DriveType;
  frontLeftPower: Signal;
  backLeftPower: Signal;
  frontRightPower: Signal;
  backRightPower: Signal;
  voltages: Signal;
  perpEncPositions: Signal[];
  perpEncVels: Signal[];
};

export async function loadLateralRampRegression(data: LateralRampData) {
  const perpEncVels = data.perpEncVels.flatMap((vs, i) =>
    fixVels(vs.times.slice(0, -1), data.perpEncPositions[i].values.slice(0, -1), vs.values.slice(0, -1)));
  const appliedVoltages = data.perpEncVels.flatMap(() => {
    const voltages = data.voltages.values.slice(0, -1).map((v, i) =>
      // TODO: Why use max here over a sum? The times are all off anyway.
      Math.max(
        -data.frontLeftPower.values[i],
        data.backLeftPower.values[i],
        data.frontRightPower.values[i],
        -data.backRightPower.values[i],
        0,
      ) * v);

    return voltages.slice(1, voltages.length - 1);
  });

  const setParams = await (async () => {
    const setData = await newLinearRegressionChart(
      document.getElementById('rampChart')!,
      [], [],
      { title: 'Lateral Ramp Regression', slope: 'lateral in per tick' },
    );

    return (inPerTick: number, kV: number, kS: number) => {
      const expectedPerpVelTicks = appliedVoltages.map(voltage => (voltage - kS) / kV);
      const perpEncVelsInches = perpEncVels.map(v => v * inPerTick);
      setData(expectedPerpVelTicks, perpEncVelsInches);
    };
  })();

  const inPerTickInput = document.getElementById('inPerTick')! as HTMLInputElement;
  const kvInput = document.getElementById('kv')! as HTMLInputElement;
  const ksInput = document.getElementById('ks')! as HTMLInputElement;
  document.getElementById('update')!.addEventListener('click', () => {
    setParams(parseFloat(inPerTickInput.value), parseFloat(kvInput.value), parseFloat(ksInput.value));
  });

  setParams(parseFloat(inPerTickInput.value), parseFloat(kvInput.value), parseFloat(ksInput.value));
}

export function installButtonHandlers<T>(name: string, loadRegression: (data: T) => Promise<void>) {
  const latestButton = document.getElementById('latest')!;
  latestButton.addEventListener('click', function () {
    fetch(`/tuning/${name}/latest.json`)
      .then(res => {
        if (res.ok) {
          const filename = res.headers.get('X-Filename');

          const a = document.createElement('a');
          a.innerText = 'Download';
          a.href = `/tuning/${name}/${filename}`;
          a.download = `${name}-${filename}`;

          const download = document.getElementById('download')!;
          download.innerHTML = '';
          download.appendChild(a);

          return res.json();
        } else {
          document.getElementById('rampChart')!.innerText = 'No data files found';
          throw new Error();
        }
      })
      .then(loadRegression)
      .catch(console.log.bind(console));
  });

  // TODO: is there not a better way to type this?
  const browseInput = document.getElementById('browse')! as (HTMLInputElement & { files: Blob[] });
  browseInput.addEventListener('change', function () {
    const reader = new FileReader();
    reader.onload = async function () {
      if (typeof reader.result === 'string') {
        await loadRegression(JSON.parse(reader.result.trim()));
      }
    };

    reader.readAsText(browseInput.files[0]);
  });
}

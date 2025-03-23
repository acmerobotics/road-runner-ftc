import Plotly from 'plotly.js-basic-dist-min';
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
    throw new Error(`${ts.length} !== ${xs.length} !== ${vs.length}`);
  }

  return numDerivOffline(ts, xs).map((est, i) => inverseOverflow(vs[i + 1], est));
}

type RegressionOptions = {
  title: string;
  slope: string;
  intercept?: string;
  xLabel: string;
  yLabel: string;
};

// data comes in pairs
async function newLinearRegressionChart(container: HTMLElement, xs: number[], ys: number[], options: RegressionOptions,
  onChange?: (m: number, b: number) => void): Promise<(xs: number[], ys: number[]) => void> {
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
    xaxis: {
      title: {
        text: options.xLabel || '',
      }
    },
    yaxis: {
      title: {
        text: options.yLabel || '',
      }
    }
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

    // This appears to be broken on recent Plotly version. I didn't see any issues online yet.
    Plotly.restyle(chartDiv, {
      'selectedpoints': [null],
    }, [0]);

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
  angVels: Signal[]; // length 3
};
type InputAngularRampData = AngularRampData & {
  // default to true
  leftEncFixVels: boolean[] | undefined;
  rightEncFixVels: boolean[] | undefined;
  parEncFixVels: boolean[] | undefined;
  perpEncFixVels: boolean[] | undefined;
};

// We skip the last measurement because it may be corrupted. Perhaps an artifact of
// poisoned Lynx modules?

function getPosZAngVelocity(data: AngularRampData) {
  const p = data.angVels.reduce<[number, number, boolean, number[]]>((acc, vsArg, axisIdx) => {
    const vs = vsArg.values;
    const maxV = vs.reduce((acc, v) => Math.max(acc, v), 0);
    const minV = vs.reduce((acc, v) => Math.max(acc, v), 0);
    const [accMaxV, _axisIdx, _axisRev, _] = acc;
    if (maxV >= -minV) {
      if (maxV >= accMaxV) {
        return [maxV, axisIdx, false, vs];
      }
    } else {
      if (-minV >= accMaxV) {
        return [-minV, axisIdx, true, vs];
      }
    }
    return acc;
  }, [0, -1, false, []]);
  if (p[1] !== 2 || p[2] !== false) {
    throw new Error(`More rotation about the ${p[2] ? '-' : '+'}${['x', 'y', 'z'][p[1]]}-axis than the +z-axis. Fix the IMU orientation and run again.`);
  }
  return p[3];
}

function sliceSignal(s: Signal, start: number, end: number): Signal {
  return {
    times: s.times.slice(start, end),
    values: s.values.slice(start, end),
  };
}

function sliceSignals(xs: Signal[], start: number, end: number): Signal[] {
  return xs.map(s => sliceSignal(s, start, end));
}

// drops the first sample and the last two samples
function prepareEncSignals(pss: Signal[], vss: Signal[], fs: boolean[]): [Signal[], Signal[]] {
  const newPss = [];
  const newVss = [];
  for (let i = 0; i < pss.length; i++) {
    const ps = sliceSignal(pss[i], 0, -1);
    const vs = sliceSignal(vss[i], 0, -1);
    const newPs = sliceSignal(ps, 1, -1);
    newPss.push(newPs);
    if (vs.times.length === 0) {
      newVss.push({
        times: newPs.times,
        values: numDerivOffline(ps.times, ps.values),
      });
    } else if (fs[i]) {
      newVss.push({
        times: newPs.times,
        values: fixVels(vs.times, ps.values, vs.values),
      });
    } else {
      newVss.push(sliceSignal(vs, 1, -1));
    }
  }
  return [newPss, newVss];
}

export function prepareAngularRampData(inputData: InputAngularRampData): AngularRampData {
  const leftEncFixVels = inputData.leftEncFixVels ?? inputData.leftEncPositions.map(() => true);
  const rightEncFixVels = inputData.rightEncFixVels ?? inputData.rightEncPositions.map(() => true);
  const parEncFixVels = inputData.parEncFixVels ?? inputData.parEncPositions.map(() => true);
  const perpEncFixVels = inputData.perpEncFixVels ?? inputData.perpEncPositions.map(() => true);

  const [leftEncPositions, leftEncVels] = prepareEncSignals(inputData.leftEncPositions, inputData.leftEncVels, leftEncFixVels);
  const [rightEncPositions, rightEncVels] = prepareEncSignals(inputData.rightEncPositions, inputData.rightEncVels, rightEncFixVels);
  const [parEncPositions, parEncVels] = prepareEncSignals(inputData.parEncPositions, inputData.parEncVels, parEncFixVels);
  const [perpEncPositions, perpEncVels] = prepareEncSignals(inputData.perpEncPositions, inputData.perpEncVels, perpEncFixVels);

  const data: AngularRampData = {
    type: inputData.type,
    leftPowers: sliceSignals(inputData.leftPowers, 1, -2),
    rightPowers: sliceSignals(inputData.rightPowers, 1, -2),
    voltages: sliceSignal(inputData.voltages, 1, -2),
    leftEncPositions,
    leftEncVels,
    rightEncPositions,
    rightEncVels,
    parEncPositions,
    parEncVels,
    perpEncPositions,
    perpEncVels,
    angVels: sliceSignals(inputData.angVels, 1, -2),
  };

  const lengths = [
    ...data.leftPowers.map(ps => ps.values.length),
    ...data.rightPowers.map(ps => ps.values.length),
    data.voltages.values.length,
    ...data.leftEncPositions.map(ps => ps.values.length),
    ...data.rightEncPositions.map(ps => ps.values.length),
    ...data.parEncPositions.map(ps => ps.values.length),
    ...data.perpEncPositions.map(ps => ps.values.length),
    ...data.leftEncVels.map(ps => ps.values.length),
    ...data.rightEncVels.map(ps => ps.values.length),
    ...data.parEncVels.map(ps => ps.values.length),
    ...data.perpEncVels.map(ps => ps.values.length),
    ...data.angVels.map(ps => ps.values.length),
  ];
  if (lengths.some(l => l !== lengths[0])) {
    throw new Error(`Lengths do not match: ${lengths.join(', ')}`);
  }

  return data;
}

// TODO: should this be tied to particular IDs?
export async function loadDeadWheelAngularRampRegression(inputData: InputAngularRampData): Promise<string[]> {
  const data = prepareAngularRampData(inputData);
  const angVels = getPosZAngVelocity(data);

  const deadWheelCharts = document.getElementById('deadWheelCharts')!;
  data.parEncVels.forEach((vs, i) => {
    const div = document.createElement('div');
    newLinearRegressionChart(div, angVels, vs.values,
      {
        title: `Parallel Wheel ${i} Regression`, slope: 'y-position',
        xLabel: 'angular velocity [rad/s]', yLabel: 'wheel velocity [ticks/s]'
      });
    deadWheelCharts.appendChild(div);
  });
  data.perpEncVels.forEach((vs, i) => {
    const div = document.createElement('div');
    newLinearRegressionChart(div, angVels, vs.values,
      {
        title: `Perpendicular Wheel ${i} Regression`, slope: 'x-position',
        xLabel: 'angular velocity [rad/s]', yLabel: 'wheel velocity [ticks/s]'
      });
    deadWheelCharts.appendChild(div);
  });

  const setParams = await (async () => {
    const allPowers = [...data.leftPowers, ...data.rightPowers];
    const appliedVoltages = data.voltages.values.map((v, i) =>
      allPowers.reduce((acc, ps) => Math.max(acc, ps.values[i]), 0) * v);

    const setTrackWidthData = await newLinearRegressionChart(
      document.getElementById('trackWidthChart')!,
      [], [],
      { title: 'Track Width Regression', slope: 'track width', xLabel: 'angular velocity [rad/s]', yLabel: 'wheel velocity [ticks/s]' },
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

  return [];
}

export async function loadDriveEncoderAngularRampRegression(inputData: InputAngularRampData): Promise<string[]> {
  const data = prepareAngularRampData(inputData);

  await newLinearRegressionChart(
    document.getElementById('rampChart')!,
    [
      ...data.leftEncVels.flatMap(vs => vs.values.map(v => -v)),
      ...data.rightEncVels.flatMap(vs => vs.values),
    ],
    [
      ...data.leftPowers.flatMap(ps => {
        return ps.values.map((p, i) => -p * data.voltages.values[i]);
      }),
      ...data.rightPowers.flatMap(ps => {
        return ps.values.map((p, i) => p * data.voltages.values[i]);
      }),
    ],
    {
      title: 'Feedforward Regression', slope: 'kV', intercept: 'kS',
      xLabel: 'wheel velocity [ticks/s]', yLabel: 'applied voltage [V]'
    },
  );

  const angVels = getPosZAngVelocity(data).slice(1, -1);

  await newLinearRegressionChart(
    document.getElementById('trackWidthChart')!,
    angVels,
    angVels.map((_, i) =>
      (data.leftEncVels.reduce((acc, vs) => acc - vs.values[i], 0) / data.leftEncVels.length
        + data.rightEncVels.reduce((acc, vs) => acc + vs.values[i], 0) / data.rightEncVels.length)
      * (data.type === 'mecanum' ? 0.5 : 1)
    ),
    { title: 'Track Width Regression', slope: 'track width', xLabel: 'angular velocity [rad/s]', yLabel: 'wheel velocity [ticks/s]' },
  );

  return [];
}

type ForwardRampData = {
  type: DriveType;
  powers: Signal[];
  voltages: Signal;
  forwardEncPositions: Signal[];
  forwardEncVels: Signal[];
};
type InputForwardRampData = ForwardRampData & {
  forwardEncFixVels: boolean[] | undefined;
};

export function prepareForwardRampData(inputData: InputForwardRampData): ForwardRampData {
  const forwardEncFixVels = inputData.forwardEncFixVels ?? inputData.forwardEncVels.map(() => true);
  const [forwardEncPositions, forwardEncVels] = prepareEncSignals(inputData.forwardEncPositions, inputData.forwardEncVels, forwardEncFixVels);

  const data: ForwardRampData = {
    type: inputData.type,
    powers: sliceSignals(inputData.powers, 1, -2),
    voltages: sliceSignal(inputData.voltages, 1, -2),
    forwardEncPositions,
    forwardEncVels,
  };

  const lengths = [
    ...data.powers.map(ps => ps.values.length),
    data.voltages.values.length,
    ...data.forwardEncPositions.map(ps => ps.values.length),
    ...data.forwardEncVels.map(vs => vs.values.length),
  ]
  if (lengths.some(l => l !== lengths[0])) {
    throw new Error(`Lengths do not match: ${lengths.join(', ')}`);
  }

  return data;
}

export async function loadForwardRampRegression(inputData: InputForwardRampData): Promise<string[]> {
  const data = prepareForwardRampData(inputData);

  const appliedVoltages = data.forwardEncVels.flatMap(() =>
    data.voltages.values.map((v, i) =>
      data.powers.reduce((acc, ps) => Math.max(acc, ps.values[i]), 0) * v));

  await newLinearRegressionChart(
    document.getElementById('rampChart')!,
    data.forwardEncVels.flatMap(v => v.values), appliedVoltages,
    { title: 'Forward Ramp Regression', slope: 'kV', intercept: 'kS', xLabel: 'forward velocity [ticks/s]', yLabel: 'applied voltage [V]' },
  );

  return [];
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
type InputLateralRampData = LateralRampData & {
  perpEncFixVels: boolean[] | undefined;
};

export function prepareLateralRampData(inputData: InputLateralRampData): LateralRampData {
  const perpEncFixVels = inputData.perpEncFixVels ?? inputData.perpEncVels.map(() => true);
  const [perpEncPositions, perpEncVels] = prepareEncSignals(inputData.perpEncPositions, inputData.perpEncVels, perpEncFixVels);

  const data: LateralRampData = {
    type: inputData.type,
    frontLeftPower: sliceSignal(inputData.frontLeftPower, 1, -2),
    backLeftPower: sliceSignal(inputData.backLeftPower, 1, -2),
    frontRightPower: sliceSignal(inputData.frontRightPower, 1, -2),
    backRightPower: sliceSignal(inputData.backRightPower, 1, -2),
    voltages: sliceSignal(inputData.voltages, 1, -2),
    perpEncPositions: perpEncPositions,
    perpEncVels: perpEncVels,
  };

  const lengths = [
    data.frontLeftPower.values.length,
    data.backLeftPower.values.length,
    data.frontRightPower.values.length,
    data.backRightPower.values.length,
    data.voltages.values.length,
    ...perpEncPositions.map(p => p.values.length),
    ...perpEncVels.map(v => v.values.length),
  ];
  if (lengths.some(l => l !== lengths[0])) {
    throw new Error(`Lengths do not match: ${lengths.join(', ')}`);
  }

  return data;
}

export async function loadLateralRampRegression(inputData: InputLateralRampData): Promise<string[]> {
  const data = prepareLateralRampData(inputData);

  const appliedVoltages = data.perpEncVels.flatMap(() =>
    data.voltages.values.map((v, i) =>
      // TODO: Why use max here over a sum? The times are all off anyway.
      Math.max(
        -data.frontLeftPower.values[i],
        data.backLeftPower.values[i],
        data.frontRightPower.values[i],
        -data.backRightPower.values[i],
        0,
      ) * v));
  const allPerpEncVels = data.perpEncVels.flatMap(v => v.values);

  const setParams = await (async () => {
    const setData = await newLinearRegressionChart(
      document.getElementById('rampChart')!,
      [], [],
      { title: 'Lateral Ramp Regression', slope: 'lateral in per tick', xLabel: 'expected velocity from feedforward [ticks/s]', yLabel: 'actual velocity [in/s]' },
    );

    return (inPerTick: number, kV: number, kS: number) => {
      const expectedPerpVelTicks = appliedVoltages.map(voltage => (voltage - kS) / kV);
      const perpEncVelsInches = allPerpEncVels.map(v => v * inPerTick);
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

  const minVel = allPerpEncVels.reduce((acc, v) => Math.min(acc, v), 0);
  const maxAbsVel = allPerpEncVels.reduce((acc, v) => Math.max(acc, Math.abs(v)), 0);
  if (-minVel > 0.2 * maxAbsVel) {
    return ['Warning: Lateral velocity should not go negative. Make sure the robot is pushed rightward and the encoders are oriented correctly.'];
  }

  return [];
}

export function installButtonHandlers<T>(name: string, loadRegression: (data: T) => Promise<string[]>) {
  const latestButton = document.getElementById('latest')!;
  latestButton.addEventListener('click', async function () {
    document.getElementById('error')!.innerText = "";
    document.getElementById('warning')!.innerText = "";
    try {
      const res = await fetch(`/tuning/${name}/latest.json`);

      if (res.ok) {
        const filename = res.headers.get('X-Filename');

        const a = document.createElement('a');
        a.innerText = 'Download';
        a.href = `/tuning/${name}/${filename}`;
        a.download = `${name}-${filename}`;

        const download = document.getElementById('download')!;
        download.innerHTML = '';
        download.appendChild(a);

        const json = await res.json();

        const warnings = await loadRegression(json);
        if (warnings.length > 0) {
          document.getElementById('warning')!.innerText = warnings.join('\n');
        }
      } else {
        throw new Error("No data files found");
      }
    } catch (err: unknown) {
      console.error(err);
      if (typeof err === 'string') {
        document.getElementById('error')!.innerText = err;
      } else if (err instanceof Error) {
        if (import.meta.env.DEV) {
          document.getElementById('error')!.innerText = `${err}\n${err.stack}`;
        } else {
          document.getElementById('error')!.innerText = err.message;
        }
      }
    }
  });

  // TODO: is there not a better way to type this?
  const browseInput = document.getElementById('browse')! as (HTMLInputElement & { files: Blob[] });
  browseInput.addEventListener('change', function () {
    const reader = new FileReader();
    reader.onload = async function () {
      if (typeof reader.result === 'string') {
        document.getElementById('error')!.innerText = "";
        document.getElementById('warning')!.innerText = "";
        try {
          const warnings = await loadRegression(JSON.parse(reader.result.trim()));
          if (warnings.length > 0) {
            document.getElementById('warning')!.innerText = warnings.join('\n');
          }
        } catch (err: unknown) {
          console.error(err);
          if (typeof err === 'string') {
            document.getElementById('error')!.innerText = err;
          } else if (err instanceof Error) {
            if (import.meta.env.DEV) {
              document.getElementById('error')!.innerText = `${err}\n${err.stack}`;
            } else {
              document.getElementById('error')!.innerText = err.message;
            }
          }
        }
      }
    };

    reader.readAsText(browseInput.files[0]);
  });
}

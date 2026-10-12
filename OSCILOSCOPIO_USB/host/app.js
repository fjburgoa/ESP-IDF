"use strict";

const REPORT_MPU = 1;
const REPORT_BNO = 2;
const REPORT_SETTINGS = 3;
const REPORT_ORIENTATION = 4;
const MPU_PAYLOAD_BYTES = 48;
const BNO_PAYLOAD_BYTES = 60;
const BNO_VECTOR_INVALID = -32768;
const BNO_VECTOR_LSB_PER_MS2 = 100;
const G0 = 9.80665;
const BNO_MODE_NAMES = ["AMG", "NDOF", "IMUPLUS", "NDOF_FMC_OFF"];

let device = null;
let mpuTargetPitch = 0, mpuTargetRoll = 0, mpuShownPitch = 0, mpuShownRoll = 0;
let bnoTargetPitch = 0, bnoTargetRoll = 0, bnoShownPitch = 0, bnoShownRoll = 0;
let pitchOffset = 0;
let mpuTurnRate = 0, mpuBallAngle = 0;
let bnoTurnRate = 0, bnoBallAngle = 0;
let mountMode = 0, orientationPending = false, requestedMountMode = null;
let orientationTimer = null;
let mpuPackets = 0, bnoPackets = 0, lastFrame = performance.now();
let recording = false, recordedRows = [], recordingStart = 0;
let pendingMpuSample = null, pendingBnoSample = null;

const $ = (id) => document.getElementById(id);
const mpuHorizon = $("mpuHorizon");
const bnoHorizon = $("bnoHorizon");
const mpuCtx = mpuHorizon.getContext("2d");
const bnoCtx = bnoHorizon.getContext("2d");

function setStatus(connected) {
  $("status").classList.toggle("connected", connected);
  $("status").querySelector("span").textContent = connected ? "Conectado" : "Desconectado";
  $("connect").disabled = connected;
  $("disconnect").disabled = !connected;
  $("offsetDown").disabled = !connected;
  $("offsetUp").disabled = !connected;
  $("orientationToggle").disabled = !connected;
  $("startRecording").disabled = !connected || recording;
  $("stopRecording").disabled = !recording;
  if (!connected) $("bnoMode").textContent = "—";
}

function message(text = "") { $("message").textContent = text; }

async function connect(ask = true) {
  if (!("hid" in navigator)) return message("WebHID no está disponible. Usa Chrome o Edge desde localhost o HTTPS.");
  try {
    const known = await navigator.hid.getDevices();
    device = known.find(d => d.productName === "EFIS USB IMU") ?? null;
    if (!device && ask) device = (await navigator.hid.requestDevice({filters:[]}))[0] ?? null;
    if (!device) return;
    if (!device.opened) await device.open();
    device.addEventListener("inputreport", receiveReport);
    setStatus(true); message("");
  } catch (error) { message(error.name === "NotFoundError" ? "No se seleccionó ningún dispositivo." : error.message); }
}

async function disconnect() {
  if (device) {
    device.removeEventListener("inputreport", receiveReport);
    if (device.opened) await device.close();
  }
  device = null;
  orientationPending = false;
  requestedMountMode = null;
  clearTimeout(orientationTimer);
  setStatus(false);
}

function receiveReport(event) {
  if (event.reportId !== REPORT_MPU && event.reportId !== REPORT_BNO) return;
  const requiredBytes = event.reportId === REPORT_BNO ? BNO_PAYLOAD_BYTES : MPU_PAYLOAD_BYTES;
  if (event.data.byteLength < requiredBytes) return;
  const d = event.data;
  const sample = {
    sequence:d.getUint32(0,true), time:d.getUint32(4,true),
    pitch:d.getFloat32(8,true), roll:d.getFloat32(12,true),
    ax:d.getFloat32(16,true), ay:d.getFloat32(20,true), az:d.getFloat32(24,true),
    p:d.getFloat32(28,true), q:d.getFloat32(32,true), r:d.getFloat32(36,true),
    offset:d.getFloat32(40,true),
    mount:d.getUint32(44,true) & 0xFF,
    bnoMode:(d.getUint32(44,true) >>> 8) & 0xFF
  };
  if (!Object.values(sample).every(Number.isFinite)) return;
  if (event.reportId === REPORT_BNO) {
    sample.linearAx = decodeBnoVector(d.getInt16(48,true));
    sample.linearAy = decodeBnoVector(d.getInt16(50,true));
    sample.linearAz = decodeBnoVector(d.getInt16(52,true));
    sample.gravityX = decodeBnoVector(d.getInt16(54,true));
    sample.gravityY = decodeBnoVector(d.getInt16(56,true));
    sample.gravityZ = decodeBnoVector(d.getInt16(58,true));
  }
  updateValues(event.reportId === REPORT_MPU ? "mpuData" : "bnoData", sample);
  recordSample(event.reportId, sample);
  updateOrientation(sample.mount);
  updateBnoMode(sample.bnoMode);

  if (event.reportId === REPORT_MPU) {
    mpuPackets++;
    pitchOffset = sample.offset;
    mpuTargetPitch = clamp(sample.pitch + pitchOffset, -90, 90);
    mpuTargetRoll = clamp(sample.roll, -180, 180);
    mpuTurnRate = clamp(sample.r, -6, 6);
    mpuBallAngle = clamp(Math.asin(clamp(sample.ax / G0, -1, 1)) * 180 / Math.PI, -25, 25);
    $("offsetValue").textContent = `${Math.round(pitchOffset)}°`;
    updateCoordinator("mpu", mpuTurnRate, mpuBallAngle);
  } else {
    bnoPackets++;
    bnoTargetPitch = clamp(sample.pitch + sample.offset, -90, 90);
    bnoTargetRoll = clamp(sample.roll, -180, 180);
    bnoTurnRate = clamp(sample.r, -6, 6);
    bnoBallAngle = clamp(Math.asin(clamp(sample.ax / G0, -1, 1)) * 180 / Math.PI, -25, 25);
    $("bnoOffsetValue").textContent = `${Math.round(sample.offset)}°`;
    updateCoordinator("bno", bnoTurnRate, bnoBallAngle);
  }
}

function updateValues(id, s) {
  const items = [["Pitch",s.pitch,"°"],["Roll",s.roll,"°"],["ax",s.ax,"m/s²"],["ay",s.ay,"m/s²"],["az",s.az,"m/s²"],["p",s.p,"°/s"],["q",s.q,"°/s"],["r",s.r,"°/s"]];
  if (id === "bnoData") {
    items.push(
      ["Linear ax",s.linearAx,"m/s²"], ["Linear ay",s.linearAy,"m/s²"], ["Linear az",s.linearAz,"m/s²"],
      ["Gravity x",s.gravityX,"m/s²"], ["Gravity y",s.gravityY,"m/s²"], ["Gravity z",s.gravityZ,"m/s²"]
    );
  }
  $(id).innerHTML = items.map(([n,v,u]) => {
    const text = Number.isFinite(v) ? `${v.toFixed(2)} ${u}` : "—";
    return `<div class="value"><small>${n}</small><strong>${text}</strong></div>`;
  }).join("");
}

function decodeBnoVector(raw) {
  return raw === BNO_VECTOR_INVALID ? null : raw / BNO_VECTOR_LSB_PER_MS2;
}

function updateBnoMode(mode) {
  $("bnoMode").textContent = BNO_MODE_NAMES[mode] ?? `DESCONOCIDO (${mode})`;
}

function startRecording() {
  if (!device?.opened || recording) return;
  recordedRows = [];
  pendingMpuSample = null;
  pendingBnoSample = null;
  recordingStart = performance.now();
  recording = true;
  $("startRecording").disabled = true;
  $("startRecording").classList.add("recording");
  $("stopRecording").disabled = false;
  $("recordStatus").parentElement.classList.add("recording");
  updateRecordingStatus();
  message("");
}

function recordSample(reportId, sample) {
  if (!recording) return;

  if (reportId === REPORT_MPU) pendingMpuSample = sample;
  else if (reportId === REPORT_BNO) pendingBnoSample = sample;

  // Se genera una fila solamente cuando hay una muestra nueva de cada sensor.
  // Si llega dos veces el mismo sensor, conservamos su muestra mas reciente.
  if (!pendingMpuSample || !pendingBnoSample) return;

  const mpu = pendingMpuSample;
  const bno = pendingBnoSample;
  recordedRows.push([
    new Date().toISOString(),
    (performance.now() - recordingStart).toFixed(3),
    mpu.mount === 0 ? "vertical" : "horizontal",
    mpu.sequence,
    mpu.time,
    mpu.pitch.toFixed(6),
    mpu.roll.toFixed(6),
    mpu.ax.toFixed(6),
    mpu.ay.toFixed(6),
    mpu.az.toFixed(6),
    mpu.p.toFixed(6),
    mpu.q.toFixed(6),
    mpu.r.toFixed(6),
    mpu.offset.toFixed(3),
    bno.sequence,
    bno.time,
    bno.pitch.toFixed(6),
    bno.roll.toFixed(6),
    bno.ax.toFixed(6),
    bno.ay.toFixed(6),
    bno.az.toFixed(6),
    bno.p.toFixed(6),
    bno.q.toFixed(6),
    bno.r.toFixed(6),
    bno.offset.toFixed(3),
    BNO_MODE_NAMES[bno.bnoMode] ?? `UNKNOWN_${bno.bnoMode}`
  ]);
  pendingMpuSample = null;
  pendingBnoSample = null;
}

function updateRecordingStatus() {
  const elapsed = recording ? (performance.now() - recordingStart) / 1000 : 0;
  $("recordStatus").textContent = recording
    ? `Registrando · ${recordedRows.length} pares · ${elapsed.toFixed(1)} s`
    : `Registro detenido · ${recordedRows.length} pares`;
}

function stopRecording() {
  if (!recording) return;
  recording = false;
  $("startRecording").disabled = !device?.opened;
  $("startRecording").classList.remove("recording");
  $("stopRecording").disabled = true;
  $("recordStatus").parentElement.classList.remove("recording");
  updateRecordingStatus();

  const header = [
    "host_time_iso", "host_elapsed_ms", "orientation",
    "mpu_sequence", "mpu_esp_time_ms", "mpu_pitch_deg", "mpu_roll_deg",
    "mpu_ax_ms2", "mpu_ay_ms2", "mpu_az_ms2",
    "mpu_p_dps", "mpu_q_dps", "mpu_r_dps", "mpu_pitch_offset_deg",
    "bno_sequence", "bno_esp_time_ms", "bno_pitch_deg", "bno_roll_deg",
    "bno_ax_ms2", "bno_ay_ms2", "bno_az_ms2",
    "bno_p_dps", "bno_q_dps", "bno_r_dps", "bno_pitch_offset_deg",
    "bno_operation_mode"
  ];
  const csv = [header, ...recordedRows]
    .map(row => row.map(csvField).join(","))
    .join("\r\n");
  const blob = new Blob(["\uFEFF", csv], {type:"text/csv;charset=utf-8"});
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  const stamp = new Date().toISOString().replace(/[:.]/g, "-");
  link.href = url;
  link.download = `efis_usb_${stamp}.csv`;
  document.body.appendChild(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function csvField(value) {
  const text = String(value);
  return /[",\r\n]/.test(text) ? `"${text.replaceAll('"','""')}"` : text;
}

async function changeOffset(delta) {
  if (!device?.opened) return;
  const next = clamp(Math.round(pitchOffset) + delta, -30, 30);
  const data = new ArrayBuffer(4);
  new DataView(data).setInt32(0, next, true);
  try { await device.sendReport(REPORT_SETTINGS, new Uint8Array(data)); }
  catch (error) { message(`No se pudo guardar el ajuste: ${error.message}`); }
}

function updateOrientation(receivedMode) {
  if (receivedMode !== 0 && receivedMode !== 1) return;
  mountMode = receivedMode;
  if (orientationPending && receivedMode !== requestedMountMode) return;
  orientationPending = false;
  requestedMountMode = null;
  clearTimeout(orientationTimer);
  const horizontal = mountMode === 1;
  const button = $("orientationToggle");
  button.textContent = `Orientación: ${horizontal ? "horizontal" : "vertical"}`;
  button.classList.toggle("horizontal", horizontal);
  button.disabled = !device?.opened;
}

async function toggleOrientation() {
  if (!device?.opened || orientationPending) return;
  const requested = mountMode === 0 ? 1 : 0;
  const button = $("orientationToggle");
  orientationPending = true;
  requestedMountMode = requested;
  button.disabled = true;
  button.textContent = "Cambiando orientación…";
  try {
    await device.sendReport(REPORT_ORIENTATION, new Uint8Array([requested]));
    orientationTimer = setTimeout(() => {
      if (!orientationPending) return;
      orientationPending = false;
      requestedMountMode = null;
      updateOrientation(mountMode);
      message("El ESP32-S3 no confirmó el cambio de orientación.");
    }, 2000);
  } catch (error) {
    orientationPending = false;
    requestedMountMode = null;
    button.disabled = false;
    updateOrientation(mountMode);
    message(`No se pudo cambiar la orientación: ${error.message}`);
  }
}

function updateCoordinator(prefix, turnRate, ballAngle) {
  const aircraftAngle = clamp(turnRate / 3 * 25, -50, 50);
  const n = clamp(ballAngle / 25, -1, 1);
  const x = n * 112, y = 18 * n * n;
  $(`${prefix}Aircraft`).setAttribute("transform", `rotate(${aircraftAngle.toFixed(2)} 400 400)`);
  $(`${prefix}Ball`).setAttribute("transform", `translate(${x.toFixed(2)} ${y.toFixed(2)})`);
  $(`${prefix}TurnValue`).textContent = `${turnRate.toFixed(2)}°/s`;
  $(`${prefix}BallValue`).textContent = `${ballAngle.toFixed(1)}°`;
}

function drawHorizon(horizon, ctx, pitch, roll) {
  const w=horizon.width,h=horizon.height,cx=w/2,cy=h/2,r=430,pxPerDeg=10;
  ctx.clearRect(0,0,w,h);
  ctx.beginPath();ctx.arc(cx,cy,480,0,Math.PI*2);ctx.fillStyle="#050608";ctx.fill();ctx.lineWidth=9;ctx.strokeStyle="#43484d";ctx.stroke();
  ctx.save();ctx.beginPath();ctx.arc(cx,cy,r,0,Math.PI*2);ctx.clip();ctx.translate(cx,cy);ctx.rotate(-roll*Math.PI/180);ctx.translate(0,pitch*pxPerDeg);
  let sky=ctx.createLinearGradient(0,-950,0,0);sky.addColorStop(0,"#0062ad");sky.addColorStop(.55,"#218fd3");sky.addColorStop(1,"#66c6ef");ctx.fillStyle=sky;ctx.fillRect(-1800,-1800,3600,1800);
  let earth=ctx.createLinearGradient(0,0,0,1000);earth.addColorStop(0,"#875635");earth.addColorStop(1,"#2c190f");ctx.fillStyle=earth;ctx.fillRect(-1800,0,3600,1800);ctx.fillStyle="#fff";ctx.fillRect(-1800,-7,3600,14);
  ctx.strokeStyle="#fff";ctx.fillStyle="#fff";ctx.lineWidth=6;ctx.textAlign="center";ctx.textBaseline="middle";ctx.font="bold 32px Arial";
  for(let a=-80;a<=80;a+=5){if(!a)continue;const y=-a*pxPerDeg,major=Math.abs(a)%10===0,half=major?90:38;ctx.beginPath();ctx.moveTo(-half,y);ctx.lineTo(half,y);ctx.stroke();if(major&&Math.abs(a)<=50){ctx.fillText(Math.abs(a),-130,y);ctx.fillText(Math.abs(a),130,y)}}ctx.restore();
  ctx.save();ctx.translate(cx,cy);ctx.rotate(-roll*Math.PI/180);ctx.fillStyle="#ffe400";ctx.beginPath();ctx.moveTo(0,-r+16);ctx.lineTo(-14,-r+48);ctx.lineTo(14,-r+48);ctx.fill();ctx.restore();
  const marks=[-60,-45,-30,-20,-10,0,10,20,30,45,60];ctx.strokeStyle="#fff";for(const a of marks){const rad=(a-90)*Math.PI/180,len=a%30===0?50:38;ctx.lineWidth=8;ctx.beginPath();ctx.moveTo(cx+Math.cos(rad)*r,cy+Math.sin(rad)*r);ctx.lineTo(cx+Math.cos(rad)*(r-len),cy+Math.sin(rad)*(r-len));ctx.stroke()}
  ctx.strokeStyle="#ffe500";ctx.fillStyle="#ffe500";ctx.lineWidth=12;ctx.lineCap="round";ctx.beginPath();ctx.moveTo(cx-235,cy);ctx.lineTo(cx-68,cy);ctx.quadraticCurveTo(cx-54,cy+42,cx,cy+42);ctx.quadraticCurveTo(cx+54,cy+42,cx+68,cy);ctx.lineTo(cx+235,cy);ctx.stroke();ctx.beginPath();ctx.arc(cx,cy,8,0,Math.PI*2);ctx.fill();
}

function animate(now) {
  const dt=Math.min((now-lastFrame)/1000,.05),alpha=1-Math.exp(-10*dt);lastFrame=now;
  mpuShownPitch+=(mpuTargetPitch-mpuShownPitch)*alpha;mpuShownRoll+=(mpuTargetRoll-mpuShownRoll)*alpha;
  bnoShownPitch+=(bnoTargetPitch-bnoShownPitch)*alpha;bnoShownRoll+=(bnoTargetRoll-bnoShownRoll)*alpha;
  drawHorizon(mpuHorizon,mpuCtx,mpuShownPitch,mpuShownRoll);
  drawHorizon(bnoHorizon,bnoCtx,bnoShownPitch,bnoShownRoll);
  $("mpuPitchValue").textContent=`${mpuShownPitch.toFixed(2)}°`;$("mpuRollValue").textContent=`${mpuShownRoll.toFixed(2)}°`;
  $("bnoPitchValue").textContent=`${bnoShownPitch.toFixed(2)}°`;$("bnoRollValue").textContent=`${bnoShownRoll.toFixed(2)}°`;
  requestAnimationFrame(animate);
}

function clamp(v,min,max){return Math.max(min,Math.min(max,v))}
$("connect").addEventListener("click",()=>connect(true));
$("disconnect").addEventListener("click",()=>disconnect().catch(e=>message(e.message)));
$("offsetDown").addEventListener("click",()=>changeOffset(-1));
$("offsetUp").addEventListener("click",()=>changeOffset(1));
$("orientationToggle").addEventListener("click",toggleOrientation);
$("startRecording").addEventListener("click",startRecording);
$("stopRecording").addEventListener("click",stopRecording);
navigator.hid?.addEventListener("disconnect",e=>{if(e.device===device)disconnect()});
setInterval(()=>{$("mpuRate").textContent=`${mpuPackets} Hz`;$('bnoRate').textContent=`${bnoPackets} Hz`;mpuPackets=bnoPackets=0;updateRecordingStatus()},1000);
updateValues("mpuData",{pitch:0,roll:0,ax:0,ay:0,az:0,p:0,q:0,r:0});updateValues("bnoData",{pitch:0,roll:0,ax:0,ay:0,az:0,p:0,q:0,r:0,linearAx:null,linearAy:null,linearAz:null,gravityX:null,gravityY:null,gravityZ:null});
updateCoordinator("mpu",0,0);updateCoordinator("bno",0,0);requestAnimationFrame(animate);connect(false);

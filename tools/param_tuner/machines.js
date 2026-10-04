// 機体ごとのパラメータの場所を決める(tx_term.js / terminal.js 用)。
// Python 側の machine_paths.py と同じ決まり:
//   1. 引数で渡した名前  2. 環境変数 EXIA_MACHINE
//   3. つないでいる基板の USB シリアル番号が登録されている機体  4. machines.yaml の default
// パラメータは machines/<機体>/profile/、登録簿は machines.yaml。
const fs = require("fs");
const path = require("path");
const yaml = require("js-yaml");

const MACHINES_YAML = path.join(__dirname, "machines.yaml");

function readRegistry() {
  if (!fs.existsSync(MACHINES_YAML)) {
    throw new Error(`${MACHINES_YAML} がありません(Param Console で機体を追加してください)`);
  }
  const doc = yaml.load(fs.readFileSync(MACHINES_YAML, "utf-8")) || {};
  const machines = (doc.machines || []).filter((m) => m && m.id);
  const ids = machines.map((m) => String(m.id));
  const def = ids.includes(doc.default) ? doc.default : ids[0] || null;
  return { default: def, machines, ids };
}

function profileDir(machine) {
  return path.join(__dirname, "machines", machine, "profile");
}

// ログ(csv)の保存先。機体ごとに machines/<機体>/logs。機体が分からなければ共通の logs/。
// 迷路(maze_logs)は全機体で共通。
function logsDir(machine) {
  return machine ? path.join(__dirname, "machines", machine, "logs") : path.join(__dirname, "logs");
}

// USB シリアル番号 → 登録されている機体(無ければ null)
function machineForSerial(serial) {
  if (!serial) return null;
  let reg;
  try {
    reg = readRegistry();
  } catch (e) {
    return null;
  }
  const m = reg.machines.find((x) => (x.serials || []).map(String).includes(serial));
  return m ? String(m.id) : null;
}

// つないでいる Pico の USB シリアル番号から機体を決める(登録されていなければ null)
async function connectedBoard() {
  const { SerialPort } = require("serialport");
  const ports = await SerialPort.list();
  const pico = ports.find((p) => p.serialNumber && /ttyACM/.test(p.path));
  if (!pico) return { serial: null, machine: null };
  const reg = readRegistry();
  const m = reg.machines.find((x) => (x.serials || []).map(String).includes(pico.serialNumber));
  return { serial: pico.serialNumber, machine: m ? String(m.id) : null };
}

async function resolveMachine(explicit) {
  const reg = readRegistry();
  if (reg.ids.length === 0) throw new Error("machines.yaml に機体がありません");
  const name = explicit || process.env.EXIA_MACHINE;
  if (name) {
    if (!reg.ids.includes(name)) {
      throw new Error(`機体 "${name}" は登録されていません(登録済み: ${reg.ids.join(", ")})`);
    }
    return name;
  }
  const board = await connectedBoard();
  if (board.machine) return board.machine;
  if (board.serial && reg.ids.length > 1) {
    // 別の機体のパラメータを黙って送らない
    throw new Error(
      `つないでいる基板 (${board.serial}) はどの機体にも登録されていません。\n` +
        `  機体を指定する(引数か EXIA_MACHINE=<機体>)か、Param Console で基板を登録してください。\n` +
        `  登録済みの機体: ${reg.ids.join(", ")}`
    );
  }
  return reg.default;
}

module.exports = { readRegistry, profileDir, logsDir, machineForSerial, connectedBoard, resolveMachine };

var fs = require("fs");
const yaml = require("js-yaml");
const { decode } = require("punycode");
const { SerialPort } = require("serialport");
const { ReadlineParser } = require("@serialport/parser-readline");
const { ByteLengthParser } = require("@serialport/parser-byte-length");
const path = require("path");

let comport;
let port;

let parser;
let binaryMode = false;


let ready = function () {
  console.log(comport)
  port = new SerialPort(
    {
      baudRate: 3000000,
      path: comport,
      highWaterMark: 256 * 1024,  // 256KB buffer (default: 64KB)
      // baudRate: 115200,
    },
    (e) => {
      if (e) {
        console.log("comport access denied, retrying...");
        setTimeout(waitForPico, 200);
      } else {
        console.log("connect");
      }
    }
  );

  port.on("close", () => {
    console.log(`\n[${new Date().toLocaleTimeString()}] 切断。再接続を待機中...`);
    setTimeout(waitForPico, 200);
  });
  port.on("error", (err) => {
    console.error("ポートエラー:", err.message);
  });

  // switchToBinaryMode(336);

  let obj = {
    dump_to_csv_ready: false,
    dump_to_map: false,
    dump_to_csv_text: false,
    data_struct: [],
    file_name: getNowYMD(),
    record: "",
  };

  switchLineMode(obj, parser);
};

const getNowYMD = () => {
  var dt = new Date();
  var y = dt.getFullYear();
  var m = ("00" + (dt.getMonth() + 1)).slice(-2);
  var d = ("00" + dt.getDate()).slice(-2);
  var h = ("00" + dt.getHours()).slice(-2);
  var M = ("00" + dt.getMinutes()).slice(-2);
  var s = ("00" + dt.getSeconds()).slice(-2);
  return `${y}${m}${d}_${h}${M}_${s}.csv`;
}
const getNowYMD_maze = () => {
  var dt = new Date();
  var y = dt.getFullYear();
  var m = ("00" + (dt.getMonth() + 1)).slice(-2);
  var d = ("00" + dt.getDate()).slice(-2);
  var h = ("00" + dt.getHours()).slice(-2);
  var M = ("00" + dt.getMinutes()).slice(-2);
  var s = ("00" + dt.getSeconds()).slice(-2);
  return `${y}${m}${d}_${h}${M}_${s}.maze`;
}

const switchLineMode = (obj) => {
  port.unpipe(parser);
  parser = port.pipe(new ReadlineParser({ delimiter: "\r\n" }));
  parser.on("data", function (data) {
    console.log(data);

    // ---- テキスト CSV モード (csv___ 〜 end___) ----
    if (obj.dump_to_csv_text) {
      if (data.match(/^end___/)) {
        obj.dump_to_csv_text = false;
        fs.writeFileSync(`${__dirname}/logs/${obj.file_name}`, obj.record, { flag: "w+" });
        fs.copyFileSync(`${__dirname}/logs/${obj.file_name}`, `${__dirname}/logs/latest.csv`);
        console.log(`[text CSV] saved: ${obj.file_name}`);
      } else {
        obj.record += `${data}\n`;
      }
      return;
    }

    if (obj.dump_to_csv_ready) {
      const d = data.split(":");
      console.log(d);
      if (d.length == 3) {
        const name = d[0];
        const type = d[1];
        const size = parseInt(d[2]);
        obj.data_struct.push({
          name: name,
          type: type,
          size: size
        });
      }
    } else if (obj.dump_to_map) {
      if (data.match(/^end___/)) {
        obj.dump_to_map = false;
        console.log(`${__dirname}/maze_logs/${obj.file_name}`);

        let maze_list = obj.record.split(",").map((e) => { return e.trim(); }).map((e) => { return (parseInt(e)); });
        let size = 16;
        if (maze_list.length > 300) {
          size = 32;
        }
        for (let y = 0; y < size; y++) {
          for (let x = 0; x < size; x++) {
            // skip excahnged point
            if (x >= y) {
              continue;
            }
            let idx = y * size + x;
            let idx2 = x * size + y;
            let tmp = maze_list[idx];
            maze_list[idx] = maze_list[idx2];
            maze_list[idx2] = tmp;
          }
        }

        fs.writeFileSync(`${__dirname}/maze_logs/${obj.file_name}`, `${maze_list.join(",")}`, {
          flag: "w+",
        });
      }
      if (obj.dump_to_map) {
        obj.record += `${data}\n`;
      }
    }

    if (data.match(/^csv___/)) {
      obj.dump_to_csv_text = true;
      obj.file_name = getNowYMD();
      obj.record = "";
      console.log(`[text CSV] start: ${obj.file_name}`);
    }

    if (data.match(/^ready___/)) {
      obj.dump_to_csv_ready = true;
      obj.file_name = getNowYMD();
      obj.record = "";
      obj.data_struct = [];
      obj.byte_size = parseInt(data.split(":")[1]);

      console.log(obj);
    }

    if (data.match(/^start___/)) {
      obj.file_name = getNowYMD();
      obj.record = "";
      obj.total_bytes = parseInt(data.split(":")[1]);

      console.log('First 20 columns:');
      for (let i = 0; i < 20 && i < obj.data_struct.length; i++) {
        console.log(`  [${i}] ${obj.data_struct[i].name}: ${obj.data_struct[i].type}`);
      }
      console.log(`total_bytes: ${obj.total_bytes}`);
      switchToBinaryMode(obj);
    }

    if (data.match(/^map___/)) {
      obj.dump_to_map = true;
      obj.file_name = getNowYMD_maze();
      obj.record = "";
      console.log(obj);
    }
  });
}

const switchToBinaryMode = (obj) => {
  const recordByteSize = obj.data_struct.reduce((s, d) => s + d.size, 0);
  const totalBytes = obj.total_bytes;
  const recordNum = Math.floor(totalBytes / recordByteSize);
  const fieldCount = obj.data_struct.length;
  console.log('record size:', recordByteSize, 'bytes');
  console.log('total bytes:', totalBytes, 'bytes');
  console.log('record count:', recordNum);
  console.log('header count:', fieldCount);

  binaryMode = true;
  port.unpipe(parser);
  parser = port.pipe(new ByteLengthParser({ length: totalBytes, highWaterMark: 256 * 1024 }));

  const header = obj.data_struct.map(d => d.name).join(',');
  console.log('header:', header);

  // フィールドごとの読み取り関数とサイズをループ外で確定
  const fieldReaders = obj.data_struct.map(d => {
    if (d.type === 'float') return (buf, off) => buf.readFloatLE(off);
    if (d.type === 'int')   return (buf, off) => buf.readInt32LE(off);
    if (d.type === 'short') return (buf, off) => buf.readInt16LE(off);
    throw new Error(`Unsupported type: ${d.type}`);
  });
  const fieldSizes = obj.data_struct.map(d => d.size);
  const indexField = obj.data_struct.findIndex(d => d.name === 'index');

  parser.on('data', (binaryData) => {
    const record = new Array(fieldCount);
    const rows = new Array(recordNum + 1);
    rows[0] = header;
    let last_idx = 0;

    for (let j = 0; j < recordNum; j++) {
      let offset = j * recordByteSize;
      for (let i = 0; i < fieldCount; i++) {
        record[i] = fieldReaders[i](binaryData, offset);
        offset += fieldSizes[i];
      }
      if (indexField >= 0) {
        const idx = record[indexField];
        if (idx % 100 === 0) console.log(last_idx, idx);
        last_idx = idx;
      }
      rows[j + 1] = record.join(',');
    }

    const content = rows.join('\n') + '\n';
    fs.writeFileSync(`${__dirname}/logs/${obj.file_name}`, content, { flag: "w+" });
    fs.copyFileSync(`${__dirname}/logs/${obj.file_name}`, `${__dirname}/logs/latest.csv`);
    console.log(`[LoggingTask] dump done: ${recordNum} records -> ${obj.file_name}`);
    switchLineMode({
      dump_to_csv_ready: false,
      dump_to_map: false,
      dump_to_csv_text: false,
      data_struct: [],
      file_name: getNowYMD(),
      record: "",
    });
  });
}

function waitForPico() {
  console.log("Pico を待機中... (終了: Ctrl+C)");
  const poll = () => {
    SerialPort.list().then(
      (ports) => {
        const found = ports.find(
          (p) =>
            (p.path.match(/usbserial/) ||
              p.path.match(/COM/) ||
              p.path.match(/ttyUSB/) ||
              p.path.match(/ttyACM/)) &&
            p.serialNumber
        );
        if (found) {
          comport = found.path;
          console.log(`[${new Date().toLocaleTimeString()}] 接続: ${comport}`);
          ready();
        } else {
          setTimeout(poll, 200);
        }
      },
      () => setTimeout(poll, 200)
    );
  };
  poll();
}

waitForPico();

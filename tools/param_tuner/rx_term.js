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
  let last_ang_sum = 0;
  const dataSize2 = obj.data_struct.reduce((prev, cur) => {
    return prev + cur.size;
  }, 0);
  const recordByteSize = dataSize2;
  const totalBytes = obj.total_bytes;
  const recordNum = Math.floor(totalBytes / recordByteSize);
  console.log('record size:', recordByteSize, 'bytes');
  console.log('total bytes:', totalBytes, 'bytes');
  console.log('record count:', recordNum);
  console.log('header count:', obj.data_struct.length);

  binaryMode = true;

  // パイプラインをクリア
  port.unpipe(parser);

  // ByteLength パーサに切り替え (全データを一括受信)
  parser = port.pipe(new ByteLengthParser({ length: totalBytes, highWaterMark: 256 * 1024 }));

  const header = obj.data_struct.map((data) => {
    return data.name
  }).join(',');
  obj.record += `${header}\n`;
  console.log('header:', header);

  let last_index = 0;
  let last_idx = 0;

  parser.on('data', (binaryData) => {

    for (let j = 0; j < recordNum; j++) {

      let offset = j * recordByteSize;
      let record = [];

      for (let i = 0; i < obj.data_struct.length; i++) {

        const data = obj.data_struct[i];
        if (data === undefined) {
          continue;
        }
        const tmp_data = offset;
        switch (data.type) {
          case 'float':
            record.push(binaryData.readFloatLE(tmp_data));
            offset += data.size;
            break;
          case 'int':
            record.push(binaryData.readInt32LE(tmp_data));
            offset += data.size;
            break;
          case 'short':
            record.push(binaryData.readInt16LE(tmp_data));
            offset += data.size;
            break;
          default:
            throw new Error(`Unsupported data type: ${data.type}`);
        }
        if (data.name === 'index') {
          const idx = parseInt(binaryData.readInt32LE(tmp_data));
          if (idx % 100 === 0) {
            console.log(last_idx, idx);
          }
          last_idx = idx;
        }
      }

      const valid = obj.data_struct.every((data, i) => {
          let res = true;
          // return true
          if (record[i] < -100000 || record[i] > 100000)
            res = false;
          if (data.name === "index") {
            if (record[i] < 0 || record[i] > 100000)
              res = false;
            if (Math.abs(record[0] - last_index) > 100)
              res = false;
          }
          if (data.name.match(/_pid_/) !== null)
            if (record[i] > 100000 || record[i] < -100000) {
              console.log(data.name, 'error:', record[i]);
              res = false;
            }
          if (data.name.match(/ff_/) !== null)
            if (record[i] > 100000 || record[i] < -100000) {
              console.log(data.name, 'error:', record[i]);
              res = false;
            }
          if (data.name === "motion_state")
            if (record[i] > 1000 || record[i] < -1000) {
              console.log(data.name, 'error:', record[i]);
              res = false;
            }
          if (data.name === "timestamp")
            if (record[i] > 10000 || record[i] < 0) {
              console.log(data.name, 'error:', record[i]);
              res = false;
            }
          if (data.name === "duty_roll")
            if (record[i] > 0.05 || record[i] < -0.05) {
              console.log(data.name, 'error:', record[i]);
              res = false;
            }
          if (data.name === "duty_roll_before")
            if (record[i] > 70 || record[i] < -70) {
              console.log(data.name, 'error:', record[i]);
              res = false;
            }
          if (data.name === "x")
            if (record[i] > 100000 || record[i] < -100000) {
              console.log(data.name, 'error:', record[i]);
              res = false;
            }
          if (data.name === "y")
            if (record[i] > 100000 || record[i] < -100000) {
              console.log(data.name, 'error:', record[i]);
              res = false;
            }
          if (data.name === "ang_kf_sum") {
            if (record[i] > 1000 || record[i] < -1000) {
              console.log(data.name, 'error:', record[i]);
              res = false;
            }
            if (Math.abs(Math.abs(record[i]) - last_ang_sum) > 40) {
              console.log(data.name, 'error:', record[i]);
              res = false;
            }
            last_ang_sum = Math.abs(record[i]);
          }
          if (data.name === "pln_time_diff")
            if (record[i] > 3000 || record[i] < 0) {
              console.log(data.name, 'error:', record[i]);
              res = false;
            }
          return res;
          if (data.name === "battery")
            if (record[i] > 15 || record[i] < 0)
              res = false;
          if (data.name === "alpha")
            if (record[i] > 100000 || record[i] < -100000)
              res = false;
          if (data.name === "m_pid_i_v")
            if (record[i] > 0.1 || record[i] < -0.1)
              res = false;
          if (data.name === "dist")
            if (record[i] > 180 * 64 || record[i] < 0)
              res = false;
          if (data.name === "dideal_ang")
            if (record[i] > 180 * 64 || record[i] < 0)
              res = false;
          if (data.name === "v_c")
            if (record[i] > 10000 || record[i] < -10000)
              res = false;
          if (data.name === "right45_2_d")
            if (record[i] > 200 || record[i] < -200)
              res = false;
          if (data.name === "right45_3_d")
            if (record[i] > 200 || record[i] < -200)
              res = false;
          if (data.name === "left45_2_d")
            if (record[i] > 200 || record[i] < -200)
              res = false;
          if (data.name === "left45_3_d")
            if (record[i] > 200 || record[i] < -200)
              res = false;
          if (data.name === "ff_duty_front")
            if (record[i] > 100 || record[i] < -100)
              res = false;
          if (data.name === "odm_y")
            if (record[i] > 1000 || record[i] < -1000)
              res = false;
          if (data.name === "kim_theta")
            if (record[i] > 400 || record[i] < -400)
              res = false;
          if (data.name === "odm_theta")
            if (record[i] > 400 || record[i] < -400)
              res = false;
          if (data.name === "knym_v")
            if (record[i] > 20000 || record[i] < -20000)
              res = false;
          if (data.name === "knym_w")
            if (record[i] > 2000 || record[i] < -2000)
              res = false;
          if (data.name === "ang_i_bias")
            if (record[i] > 3 || record[i] < -3)
              res = false;
          if (data.name === "ang_i_bias_val")
            if (record[i] > 200 || record[i] < -200)
              res = false;
          if (data.name === "left45_d_diff")
            if (record[i] > 50 || record[i] < -50)
              res = false;
          if (data.name === "left45_2_d_diff")
            if (record[i] > 50 || record[i] < -50)
              res = false;
          if (data.name === "left45_3_d_diff")
            if (record[i] > 50 || record[i] < -50)
              res = false;
          if (data.name === "right45_d_diff")
            if (record[i] > 50 || record[i] < -50)
              res = false;
          if (data.name === "right45_2_d_diff")
            if (record[i] > 50 || record[i] < -50)
              res = false;
          if (data.name === "right45_3_d_diff")
            if (record[i] > 50 || record[i] < -50)
              res = false;
          if (data.name === "duty_roll")
            if (record[i] > 0.1 || record[i] < -0.1)
              res = false;
          if (data.name === "duty_roll_before")
            if (record[i] > 0.1 || record[i] < -0.1)
              res = false;
          return res;
      });
      if (valid) {
        last_index = record[0];
        obj.record += `${record.join(',')}\n`;
      }
    }

    fs.writeFileSync(`${__dirname}/logs/${obj.file_name}`, `${obj.record}`, {
      flag: "w+",
    });
    fs.copyFileSync(
      `${__dirname}/logs/${obj.file_name}`,
      `${__dirname}/logs/latest.csv`
    );
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

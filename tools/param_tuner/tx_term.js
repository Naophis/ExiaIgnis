var fs = require("fs");
const { spawnSync } = require("child_process");
const path = require("path");
const os = require("os");

const SEND_FILE_PY = path.join(__dirname, "../../send_file.py");

function sendViaPython(localPath, remoteName) {
  const result = spawnSync("python3", [SEND_FILE_PY, "write", localPath, remoteName], {
    stdio: "inherit",
  });
  if (result.error) throw result.error;
  if (result.status !== 0) throw new Error(`send_file.py exited with ${result.status}`);
}

const convert = (filename) => filename.split(".")[0];

async function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

const callerFun = async (mode) => {
  while (true) {
    const files = fs.readdirSync(path.join(__dirname, `profile/${mode}/`));
    const list = ["system.yaml", "hardware.yaml"].concat(
      files.filter((f) => f.match(/.yaml$/) || f.match(/.maze$/))
    );

    const col = 5;
    for (let i = 0; i < list.length; ) {
      let str = "";
      for (let j = 0; j < col && list[i + j] !== undefined; j++) {
        str += `[${i + j}]: ${convert(list[i + j])}\t`;
      }
      i += col;
      console.log(str);
    }
    console.log(">");

    const str = fs.readFileSync("/dev/stdin").toString().trim();
    const idx = parseInt(str);

    if (str === "all") {
      console.log("all");
      for (const file of files) {
        if (file.match(/.yaml$/)) {
          const remoteName = file.replace("yaml", mode);
          sendViaPython(path.join(__dirname, `profile/${mode}/${file}`), remoteName);
          await sleep(250);
          console.log(`${file}, ${remoteName}: finish!!`);
        }
      }
      for (const file of ["system.yaml", "hardware.yaml"]) {
        const remoteName = file.replace("yaml", "txt");
        sendViaPython(path.join(__dirname, "profile", file), remoteName);
        await sleep(250);
        console.log(`${file}, ${remoteName}: finish!!`);
      }
    } else {
      if (typeof idx !== "number" || isNaN(idx)) {
        console.log("invalid input");
        continue;
      }
      if (idx >= list.length) {
        console.log("out of index");
        continue;
      }

      if (idx === 0 || idx === 1) {
        const file = list[idx];
        const remoteName = file.replace("yaml", "txt");
        sendViaPython(path.join(__dirname, "profile", file), remoteName);
        await sleep(800);
        console.log(`${file}, ${remoteName}: finish!!`);
      } else {
        const file = list[idx];
        const filePath = path.join(__dirname, `profile/${mode}/${file}`);

        if (file.match(/.maze$/)) {
          const txt = fs.readFileSync(filePath, { encoding: "utf-8" });
          let maze_list = txt
            .split(",")
            .map((e) => parseInt(e.trim()) | 0xf0);
          const size = maze_list.length > 300 ? 32 : 16;
          for (let y = 0; y < size; y++) {
            for (let x = 0; x < size; x++) {
              if (x >= y) continue;
              const i1 = y * size + x;
              const i2 = x * size + y;
              [maze_list[i1], maze_list[i2]] = [maze_list[i2], maze_list[i1]];
            }
          }
          const tmpFile = path.join(os.tmpdir(), `maze_${Date.now()}.txt`);
          fs.writeFileSync(tmpFile, maze_list.join(","));
          sendViaPython(tmpFile, "maze.txt");
          fs.unlinkSync(tmpFile);
          await sleep(800);
          console.log(`maze.txt: finish!!`);
        } else if (file.match(/.yaml$/)) {
          const remoteName = file.replace("yaml", mode);
          console.log(remoteName);
          sendViaPython(filePath, remoteName);
          await sleep(600);
          console.log(`${file}, ${remoteName}: finish!!`);
        }
      }
    }
  }
};

const main = (argv) => {
  const mode = argv.length > 2 ? argv[2] : "hf";
  callerFun(mode);
};

main(process.argv);

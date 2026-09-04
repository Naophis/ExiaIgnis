// Client/server で共有する AM32 まわりの定数・型。lib/serial-manager.ts は
// serialport (Node ネイティブ依存) を読み込むため、クライアント側から値として
// import できない。共有したいものはここに置く。

export const AM32_FILE = "am32.yaml";

// UI側の操作単位。"sync" は「am32.yaml送信 + AM32WRITE」で、シリアル層の
// Am32Command ("write" | "read") とは1対1ではない。
export type Am32Action = "sync" | "read";

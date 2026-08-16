#!/bin/bash
cd "$(dirname "$0")/tools/param_tuner/webapp"

if [ ! -d node_modules ]; then
    echo "依存関係をインストールしています..."
    npm install
fi

npm run dev

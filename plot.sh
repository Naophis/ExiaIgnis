# source tools/rosws/install/setup.bash
# 使い方:
#   ./plot.sh        いちばん新しいログ(どの機体かは問わない)を PlotJuggler で開く
#   ./plot.sh N      新しい方から N 番目(0 = いちばん新しい)のログを開く
# ログは機体ごとに tools/param_tuner/machines/<機体>/logs/ に保存される。共通の
# tools/param_tuner/logs/ には、機体を分ける前のログと未登録の基板のログがあり、
# latest.csv だけは「どの機体かを問わず、いちばん新しいログ」の写しになっている。
if [ $# -ne 0 ];then
    idx=$1
    if expr "$idx" : "[0-9]*$" >&/dev/null; then
        # 全機体 + 共通のログを古い順に並べ(latest.csv は写しなので除く)、後ろから N 番目を取る
        file=$(ls -rt ./tools/param_tuner/logs/*.csv ./tools/param_tuner/machines/*/logs/*.csv 2>/dev/null \
            | grep -v '/latest\.csv$' | tail -n $(($idx+1)) | head -n 1)
        if [ -z "$file" ]; then
            echo "ログがありません"
        else
            echo $file
            read -p "Press [Enter] key to resume."
            # python3 trajectory_plot.py $file &
            `plotjuggler -d $file -l ./tools/param_tuner/profile.xml`
        fi
    else
        echo "not number"
    fi

else
    # python3 trajectory_plot.py ./tools/param_tuner/logs/latest.csv &
    plotjuggler -d ./tools/param_tuner/logs/latest.csv -l ./tools/param_tuner/profile.xml
fi

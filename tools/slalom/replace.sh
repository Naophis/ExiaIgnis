#!/bin/bash
# 機体の profile(machines/<機体>/profile。EXIA_MACHINE か machines.yaml の default)
profile_dir=$(python3 ../param_tuner/machine_paths.py) || exit 1
input_file="$profile_dir/hf/t_$1.yaml"
new_txt=$(cat "./result.yaml")

echo $1

line_number=$(grep -n '# check' "$input_file" | cut -d: -f1)
if [ -n "$line_number" ]; then
  # # checkという行以下を置き換える
  sed -i "${line_number}q" "$input_file"
  echo "# $(date)" >> "$input_file"
  echo "$new_txt" >> "$input_file"
else
  echo "check行が見つかりませんでした。"
fi
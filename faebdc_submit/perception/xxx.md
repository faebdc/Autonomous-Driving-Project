
bazel run -c opt //perception:perception_evaluation_main -- --evaluation_config_dir /${Path to your config foler}
bazel run -c opt //perception:evaluation_viewer_main  -- --eval_result_file ${Evaluation result folder}/${scenario_name}.result


bazel run -c opt //perception:perception_evaluation_main -- --evaluation_config_dir /home/faebdc/auto/final/PublicCourse/perception/config
bazel run -c opt //perception:evaluation_viewer_main  -- --eval_result_file /tmp/1528280031/pony_data.result

# Additional experiment materials

## E. Pump assembly

As mentioned on the main page, two videos of the experiments are provided.

Assembly task: https://youtu.be/SqL07pA2dIA

Training of involved policies: https://youtu.be/vfTIiDHBQ-4

## F. Code generation latency

Full set of test commands: *input.txt*

Preamble used to prompt the model: *preamble.txt*

Policies: *../lmpvc_ros2_/src/lmpvc_core/lmpvc_core/policies/*

Complete latency results, one command per row: *output.csv*

Visualization: *res.svg*

### Replicating the results

Build a Docker image, or set up the workspace.

Options:

  ```
  codegen_test_cli.py -h
  ```

Start the code generation module:

  ```
  bash | tee codegen_log.txt
  ros2 run lmpvc_codegen service
  ```

Run the test sequence:

  ```
  codegen_test_cli.py input.txt output.csv -l test_log.txt -i 10
  ```

Visualize the results:

  ```
  visualize.py output.csv
  ```

## G. Command completion performance

The same test is used to get these results, by manually analyzing the log files. Most results are easier to find from *test_log.txt*, but *codegen_log.txt* is a useful reference for missing output (generation result not valid) and additional information.

Success/failure by command: *results_summary.txt*
# P5

## How to compute PIT trajectory:
```
  - Scan weld seam.
  - Place weld_seam_scan.txt (name irrelevant) into 'scans/before/unprocessed/'
  - Edit program.py, line 30, change FILE_NAME = "weld_seam_scan"      .... Notice there is no file extension.
  - in terminal run: "python3 program.py"
  - Trajectory files are located in 'trajectories/' named weld_seam_scan-trajectory-0.txt and weld_seam_scan-trajectory-1.txt
  
```

## How to compute quality report:
```
  - Scan treated weld seam with same trjectory used to compute PIT trajectory
  - Place weld_seam_scan-Quality.txt into 'scans/after/unprocessed
  - Edit quality_scan.py line 30 change FILE_NAME = "weld_seam_scan-Quality" ... Notice there is no file extension
  - in terminal run: "python3 quality_scan.py"
  - Output is graphs of quality parameters.
```

# Evidence

Each certification phase writes its measured evidence here when its
acceptance procedure is run. Evidence is the record that converts a phase
gate from LOCKED to PASSED.

Layout:

    evidence/
      phase1/    Lunar Base Camp acceptance reports (JSON + markdown)
      phase2/    Perception evaluation reports (mIoU, per-class metrics)
      phase3/    Localisation error logs
      ...        one directory per phase

Retention policy:

* JSON and markdown reports are committed — they are the certification record.
* Heavy media (video, rosbags) is ignored by git; store externally and link
  the location from the phase's report if needed.
* Never edit a generated report by hand. If a result is wrong, fix the
  system, rerun the test, and commit the new report.

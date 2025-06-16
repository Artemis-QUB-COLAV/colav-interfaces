# CRIClassification
# This is a classification of the risk presented provided
# by the cri index

uint8 UNDEFINED  # CRI collision value not defined.

uint8 NO_RISK       # 0.0: No risk of collision detected
uint8 LOW=0         # > 0.0 - 0.2: "Low risk of collision: Monitor"
uint8 MEDIUM=1      # 0.2 - 0.5: "Moderate risk of collision: Prepare maneuver"
uint8 HIGH=2        # 0.5 - 0.8: "High risk, likely conflict ahead: Begin evasive planning"
uint8 CRITICAL=3    # 0.8 - 1.0: "immenent collision: immediate action required"

uint8 classification

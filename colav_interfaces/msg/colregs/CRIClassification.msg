# CRIClassification
# This is a classification of the risk presented provided
# by the cri index

uint8 CRI_UNDEFINED=0  # CRI collision value not defined.

uint8 CRI_NO_RISK=1       # 0.0: No risk of collision detected
uint8 CRI_LOW=2         # > 0.0 - 0.2: "Low risk of collision: Monitor"
uint8 CRI_MEDIUM=3      # 0.2 - 0.5: "Moderate risk of collision: Prepare maneuver"
uint8 CRI_HIGH=4        # 0.5 - 0.8: "High risk, likely conflict ahead: Begin evasive planning"
uint8 CRI_CRITICAL=5    # 0.8 - 1.0: "immenent collision: immediate action required"

uint8 classification

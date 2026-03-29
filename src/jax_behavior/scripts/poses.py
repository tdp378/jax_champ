# jax_behavior/poses.py

# Order: rh_hip, rh_upper, rh_lower, lh_hip, lh_upper, lh_lower, 
#        lf_hip, lf_upper, lf_lower, rf_hip, rf_upper, rf_lower
POSES = {
    "sit":   [0.0, 0.85, 0.0, 0.0, 0.85, 0.0, 0.0, 0.4, 0.85, 0.0, 0.4, 0.85],
    "lay":  [0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0],
    "stand":  [0.0] * 12,
    "high_five": [0.0, 0.6, -1.2, 0.0, 0.6, -1.2, 0.0, 0.6, -1.2, 0.5, -0.5, 0.0]
}
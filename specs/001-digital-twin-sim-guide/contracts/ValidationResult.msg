# Simulation Validation Result Message

# This message contains the results of comparing simulation
# accuracy against real-world robot behavior

string validation_id          # Unique ID of the validation test
string test_description       # Description of the validation test performed
float[] simulation_data       # Array of data from the simulation run
float[] real_world_data       # Array of data from the real-world test
float similarity_score        # Score representing how similar the datasets are (0.0-1.0)
float confidence_interval     # Confidence interval of the validation result
string[] metrics_evaluated    # List of specific metrics that were compared
bool validation_passed        # True if the validation met the required threshold
string error_message          # Error description if validation failed
float tolerance_threshold     # Threshold used for validation comparison
builtin_interfaces/Time validation_timestamp  # Time when validation was performed
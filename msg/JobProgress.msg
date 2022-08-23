demo_interfaces/Header header

#### BIG PICTURE 

## Percentage of the Job Completed
float32 percentage_progress

## Number of phases that make up the job
int32 number_of_phases

## Counter identify current phase in the job sequence
int32 current_phase

#### DETAILED PICTURE

## How many atomic tasks make up a phase
int32 phase_size

## Counter representing the current task in the current phase
int32 phase_count

## Percentage of the current phase completed
float32 phase_percentage


### If necessary, or possible placeholder
builtin_interfaces/Duration time_elapsed

string progress_msg
# Creating the Data Set
Use the following commands to collect human demonstration data for a Visuomotor Policy. Then, run the following script on the host machine.
```
python3 scripts/sim_demo.py --task=TASK --robot=ROBOT --path=PATH
```
The task must be specified as `TASK` and can be one of the following: `lid` (Closing the lid), `cup` (Cup shelving), or `ladle` (Ladle reorganization). The deployment embodiment can be specified as one of the following: `abstract`, `panda`, `spot`, `google`, or `gr1`. You should also specify the path to save data as `PATH`. Collected data would be saved in `PATH/RECORDED_TIME`.

To post-process the raw demonstration file, please use the following commands.
```
python3 scripts/sim_proc.py --path=DATA_DIR --task=TASK --robot=ROBOT --mode=sync
python3 scripts/sim_proc.py --path=DATA_DIR --task=TASK --robot=ROBOT --mode=obs
python3 scripts/sim_proc.py --path=DATA_DIR --task=TASK --robot=ROBOT --mode=delta_act
python3 scripts/sim_proc.py --path=DATA_DIR --task=TASK --robot=ROBOT --mode=history_obs
python3 scripts/bc_create_dataset.py --path=DATA_DIR
```
The dataset is generated in `./data/datasets/dataset_{TASK}.hdf5`. This process also generates multiple post-processed `hdf5` files. Please use the command below to remove them.
```
rm -rf ./DATA_DIR/*/demo.hdf5
rm -rf ./DATA_DIR/*/gray_obs.hdf5
rm -rf ./DATA_DIR/*/delta_rpy_act.hdf5
rm -rf ./DATA_DIR/*/history_obs.hdf5
```
Dataset files consist of sequences of the following data structure.
```
hdf5 dataset
├── actions: 7D value
└── observation
    ├── right_gray: 128x128x1 array
    ├── left_gray: 128x128x1 array
    ├── delta_positions: 6D value
    └── delta_eulers: 6D value
```

# Training
For training a Visuomotor Policy, please use the following commands.
```
python3 scripts/bc_train.py --config=PATH_TO_CONFIG --exp=EXPERIMENT_NAME --device=DEVICE --data_path=PATH_TO_DATASET
```
The configuration at `./config/sim.json` would be used for training as the default unless you specify `PATH_TO_CONFIG`. Trained files would be saved in `./save/EXPERIMENT_NAME/TRAINING_STARTED_TIME`. You need to create or download ([link](https://utexas.box.com/s/5twb8okdnfr2uhyf4fj3bh5ohu4w3o4r)) an `hdf5`-format dataset file and specify the path to the dataset file as `PATH_TO_CONFIG`.

# Evaluation
For evaluating a Visuomotor Policy, please use the following commands.
```
python3 scripts/sim_evaluate.py --task=TASK --robot=ROBOT --seed=SEED --ckpt_path=PATH_TO_CHECKPOINT
```
Here, you must specify the path to the pre-trained checkpoint as `PATH_TO_CHECKPOINT`. The task must be specified as `TASK` and can be one of the following: `lid` (Closing the lid), `cup` (Cup shelving), or `ladle` (Ladle reorganization). The deployment embodiment, `ROBOT`, can be specified as one of the following: `abstract`, `panda`, `spot`, `google`, or `gr1`.
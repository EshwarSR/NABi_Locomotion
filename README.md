# NABi_Locomotion
Repo for Nabi Locomotion experiments done as part of the class project for the course "CP314 Robot Learning and Control"

## Folder Structure

### RL_locomotion
`RL_locomotion` folder contains all the code/model files required to train and test the walking of NABi Robot.

Code has been taken from the [ARS authors repository](https://github.com/modestyachts/ARS). And MuJoCo model files are adopted from [this](https://github.com/jinparksj/RL_NABI) repository.

Change folder to `RL_locomotion` and run `python code/nabi_ars.py` to train the model, `python code/nabi_test_policy.py` to test the model.

MujoCo model XML file and OpenAI gym environment files can be found in the `envs` folder.

"""

Code to load a policy and generate rollout data. Adapted from https://github.com/berkeleydeeprlcourse. 
Example usage:
    python run_policy.py ../trained_policies/Humanoid-v1/policy_reward_11600/lin_policy_plus.npz Humanoid-v1 --render \
            --num_rollouts 20
"""
import numpy as np
import gym
from gym.envs.registration import register
from gym.wrappers import Monitor


def initNabi():
    register(
        id='NabiSlope-v0',
        entry_point='envs.mujoco:NabiSlopeEnv',  # '''VERY IMPORTANT!!!!!''' for IMPORT
        max_episode_steps=2000,
        reward_threshold=6000.0,
        nondeterministic=True,
        kwargs={"debug": True}
    )
    return None


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--expert_policy_file', type=str)
    parser.add_argument('--envname', type=str, default="NabiSlope-v0")
    parser.add_argument('--render', type=int, default=1)
    parser.add_argument('--num_rollouts', type=int, default=1,
                        help='Number of expert rollouts')
    parser.add_argument('--std', type=float, default=0.0,
                        help='Std of the policy')
    parser.add_argument("--rollout_len", type=int, default=2000)
    parser.add_argument("--video_dir", type=str, default="./video")

    args = parser.parse_args()
    print(args)

    print('loading and building expert policy')
    # lin_policy = np.load(args.expert_policy_file)
    # lin_policy = lin_policy.items()[0][1]
    lin_policy = np.load(args.expert_policy_file, allow_pickle=True)["arr_0"]

    M = lin_policy[0]
    # mean and std of state vectors estimated online by ARS.
    mean = lin_policy[1]
    std = lin_policy[2]  # Std of the states observered in training

    env = gym.make(args.envname)
    if args.render:
        env = Monitor(env, args.video_dir,
                      video_callable=lambda episode_id: True, force=True)

    returns = []

    for i in range(args.num_rollouts):
        print('iter', i)
        obs = env.reset()
        done = False
        totalr = 0.
        steps = 0
        while not done:
            if args.render:
                env.render(mode="rgb_array")
            action_vector = np.dot(M, (obs - mean)/std)
            # zero_act = np.zeros(len(action_vector))

            obs, r, done, _ = env.step(action_vector)
            totalr += r
            steps += 1
            if steps % 100 == 0:
                print("%i/%i" % (steps, args.rollout_len), totalr)
            if steps >= args.rollout_len:
                break
        returns.append(totalr)
        print("Episode Length:", steps)

    print('returns', returns)
    print('mean return', np.mean(returns))
    print('std of return', np.std(returns))


if __name__ == '__main__':
    initNabi()
    main()

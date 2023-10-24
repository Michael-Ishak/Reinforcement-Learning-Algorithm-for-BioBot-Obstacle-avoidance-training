#include <torch/torch.h>
#include<vector>
#include<iostream>
#include<random>
#include<chrono>

using namespace::std

const int batch_Size = 64;
const int num_updates = 10;

struct State {
    double pos_x;
    double pos_y;
    double vel_x;
    double vel_y;
    double distance_to_obstacle;
    double angle_to_obstacle;
    double vel_motor1;
    double vel_motor2;
};

struct Action {
    double left_motor_speed;
    double right_motor_speed;
};

class PPOAgent {
    public:
        PPOAgent(double learning_rate, double discount_factor, double entropy_coefficient, double clip_ramge){
            this->learning_rate = learning_rate;
            this->discount_factor = discount_factor;
            this->entropy_coefficient = entropy_coefficient;
            this->clip_range = clip_range;

        // Create neural network for policy optimization
        this->actor_critic_ = ActorCritic(STATE_DIM, ACTION_DIM, learning_rate, learning_rate, discount_factor, clip_range);
}

// Define the PPO training function
    void train(std::vector<State> states, std::vector<Action> actions, std::vector<double> rewards) {
        // Compute the discounted rewards
        std::vector<double> discounted_rewards(rewards.size());
        double current_discount = 1.0;
        double current_reward = 0.0;
        for (int i = rewards.size() - 1; i >= 0; i--) {
            current_reward = rewards[i] + discount_factor * current_reward;
            discounted_rewards[i] = current_reward;
        }

        // Normalize the discounted rewards
        double reward_mean = std::accumulate(discounted_rewards.begin(), discounted_rewards.end(), 0.0) / discounted_rewards.size();
        double reward_stddev = 0.0;
        for (double reward : discounted_rewards) {
            reward_stddev += pow(reward - reward_mean, 2);
        }
        reward_stddev = sqrt(reward_stddev / discounted_rewards.size());
        for (int i = 0; i < discounted_rewards.size(); i++) {
            discounted_rewards[i] = (discounted_rewards[i] - reward_mean) / (reward_stddev + 1e-8);
        }

        // Compute the advantages
        std::vector<double> advantages(discounted_rewards.size());
        std::vector<double> values;
        for (State state : states) {
            ActionCriticOutput output = actor_critic_.get_action_critic_output(state);
            values.push_back(output.value.item<double>());
        }
        for (int i = 0; i < advantages.size(); i++) {
            advantages[i] = discounted_rewards[i] - values[i];
        }

        // Update the actor-critic network using PPO
        double clip_min = 1.0 - clip_range;
        double clip_max = 1.0 + clip_range;
        std::vector<size_t> indices(states.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), std::mt19937{std::random_device{}()});
            // Loop over the batches and perform updates
        for (int update = 0; update < num_updates; update++) {
            // Shuffle the data
            std::shuffle(indices.begin(), indices.end(), std::mt19937{std::random_device{}()});
            
            // Loop over the batches
            for (int i = 0; i < states.size(); i += batch_size) {
                // Get the batch
                int end_index = std::min(i + batch_size, (int)states.size());
                std::vector<State> batch_states(states.begin() + i, states.begin() + end_index);
                std::vector<Action> batch_actions(actions.begin() + i, actions.begin() + end_index);
                std::vector<double> batch_advantages(advantages.begin() + i, advantages.begin() + end_index);
                std::vector<double> batch_discounted_rewards(discounted_rewards.begin() + i, discounted_rewards.begin() + end_index);

                // Compute the current policy and value estimates for the batch
                std::vector<double> old_log_probs;
                std::vector<double> old_values;
                for (State state : batch_states) {
                    ActionCriticOutput output = actor_critic_.get_action_critic_output(state);
                    old_log_probs.push_back(output.log_prob.item<double>());
                    old_values.push_back(output.value.item<double>());
                }

                // Update the actor-critic network
                actor_critic_.update(batch_states, batch_actions, batch_advantages, batch_discounted_rewards, old_log_probs, old_values, entropy_coefficient, clip_range, clip_min, clip_max);
            }
        }
    }

    private:
        double learning_rate;
        double discount_factor;
        double entropy_coefficient;
        double clip_range;
        ActorCritic actor_critic_;
}

// Train the agent
std::vector<State> states;
std::vector<Action> actions;
std::vector<double> rewards;
for (int episode = 0; episode < NUM_EPISODES; episode++) {
    State state = env.reset();
    double episode_reward = 0.0;
    for (int timestep = 0; timestep < MAX_TIMESTEPS; timestep++) {
        // Get an action from the agent and take a step in the environment
        Action action = agent.get_action(state);
        State next_state;
        double reward;
        bool done;
        std::tie(next_state, reward, done) = env.step(action);

        // Add the transition to the buffers
        states.push_back(state);
        actions.push_back(action);
        rewards.push_back(reward);

        // Update the state and episode reward
        state = next_state;
        episode_reward += reward;

        // Train the agent if the episode is done or we have reached the maximum number of timesteps
        if (done || timestep == MAX_TIMESTEPS - 1) {
            agent.train(states, actions, rewards);
            states.clear();
            actions.clear();
            rewards.clear();
        }

        // Stop the episode if we have reached the maximum number of timesteps
        if (timestep == MAX_TIMESTEPS - 1) {
            std::cout << "Episode " << episode << " finished with reward " << episode_reward << std::endl;
        }
    }

    return 0;
}
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>

namespace {

constexpr const char *kRequiredAcknowledgement = "G1_ARM_ACTIONS_APPROVED";

// Action name to ID mapping for standard actions
const std::map<std::string, int32_t> kActionNameToId = {
    {"release_arm", 99},
    {"turn_back_wave", 1},
    {"blow_kiss_with_both_hands", 11},
    {"blow_kiss_with_left_hand", 12},
    {"blow_kiss_with_right_hand", 13},
    {"both_hands_up", 15},
    {"clamp", 17},
    {"high_five", 18},
    {"hug", 19},
    {"make_heart_with_both_hands", 20},
    {"make_heart_with_right_hand", 21},
    {"refuse", 22},
    {"right_hand_up", 23},
    {"ultraman_ray", 24},
    {"wave_under_head", 25},
    {"wave_above_head", 26},
    {"shake_hand", 27},
    {"box_left_hand_win", 28},
    {"box_right_hand_win", 29},
    {"box_both_hand_win", 30},
    {"right_hand_on_heart", 33},
    {"both_hands_up_deviate_right", 34},
    {"forward_push", 36},
};

// Custom action names (executed by name, not by ID)
const std::map<std::string, std::string> kCustomActions = {
    {"Waist_Drum_Dance", "Waist_Drum_Dance"},
    {"Spin_discs", "Spin_discs"},
    {"Scratch_head", "Scratch_head"},
    {"Throw_money", "Throw_money"},
};

struct Options {
  std::string network_interface;
  std::string action;
  std::string acknowledgement;
};

void PrintUsage(const char *program) {
  std::cerr << "Usage: " << program
            << " --interface IFACE --action ACTION"
               " --motion-ack G1_ARM_ACTIONS_APPROVED\n";
}

// Parses the CLI gate and required robot network parameters before SDK2 setup.
Options ParseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    const auto require_value = [&](const char *name) -> std::string {
      if (index + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + name);
      }
      return argv[++index];
    };
    if (argument == "--interface") {
      options.network_interface = require_value("--interface");
    } else if (argument == "--action") {
      options.action = require_value("--action");
    } else if (argument == "--motion-ack") {
      options.acknowledgement = require_value("--motion-ack");
    } else if (argument == "--help" || argument == "-h") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }
  if (options.network_interface.empty()) {
    throw std::runtime_error("--interface is required");
  }
  if (options.action.empty()) {
    throw std::runtime_error("--action is required");
  }
  if (options.acknowledgement != kRequiredAcknowledgement) {
    throw std::runtime_error("exact --motion-ack value required");
  }
  return options;
}

// Maps Robonix action names to the SDK2 G1ArmActionClient methods.
int RunAction(unitree::robot::g1::G1ArmActionClient &client,
              const std::string &action) {
  // Check if it's a custom action (executed by name)
  auto custom_it = kCustomActions.find(action);
  if (custom_it != kCustomActions.end()) {
    std::cout << "Executing custom action: " << action << "\n";
    return client.ExecuteAction(action);
  }

  // Check if it's a standard action (executed by ID)
  auto standard_it = kActionNameToId.find(action);
  if (standard_it != kActionNameToId.end()) {
    std::cout << "Executing standard action: " << action
              << " (id=" << standard_it->second << ")\n";
    return client.ExecuteAction(standard_it->second);
  }

  throw std::runtime_error("unsupported action: " + action);
}

// Initializes the Unitree channel, runs exactly one preset, then exits.
int Main(int argc, char **argv) {
  const Options options = ParseOptions(argc, argv);
  unitree::robot::ChannelFactory::Instance()->Init(
      0, options.network_interface);

  unitree::robot::g1::G1ArmActionClient client;
  client.Init();
  client.SetTimeout(10.0F);

  const int32_t result = RunAction(client, options.action);
  if (result != 0) {
    std::cerr << "SDK2 action " << options.action << " returned " << result
              << "\n";
    return 3;
  }
  std::cout << "SDK2 action accepted: " << options.action << "\n";
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    return Main(argc, argv);
  } catch (const std::exception &exception) {
    std::cerr << "g1_arm_action_cli: " << exception.what() << "\n";
    PrintUsage(argv[0]);
    return 2;
  }
}

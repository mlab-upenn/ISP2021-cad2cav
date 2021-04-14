#include "opt2_solver.h"

#include <libconfig.h++>

#include "utils.h"

std::vector<aco::Node> opt2_swap(
    const std::vector<aco::Node>& existing_sequence, const int i, const int k) {
    std::vector<aco::Node> new_sequence;
    for (int x = 0; x < i; x++) {
        new_sequence.emplace_back(existing_sequence[x]);
    }
    for (int x = k; x >= i; x--) {
        new_sequence.emplace_back(existing_sequence[x]);
    }
    for (int x = k + 1; x < existing_sequence.size(); x++) {
        new_sequence.emplace_back(existing_sequence[x]);
    }
    assert(new_sequence.size() == existing_sequence.size());
    return new_sequence;
}

/**
 * Runs the opt-2 algorithm for local search for improving the sequence
 * @param cost_matrix - distance matrix
 * @param sequence - sequence to update based on opt-2 algorithm
 */
void aco::run_opt2(const Eigen::MatrixXd& cost_matrix,
                   std::vector<aco::Node>& sequence) {
    // Get opt-2 algorithm parameters
    const auto params = get_opt2_params();
    if (!params.use_opt2) return;

    if (params.max_iters != -1) {
        std::cout << "Cannot set max iters yet. running opt2 till improvement "
                     "stops.\n";
    }

    auto best_fitness_value = find_fitness_values(cost_matrix, sequence);
    std::cout << "Fitness value of sequence before Opt2: " << best_fitness_value
              << std::endl;
    int I = 0;
    while (true) {
        for (int i = 1; i < sequence.size() - 2; i++) {
            for (int k = i + 1; k < sequence.size() - 1; k++) {
                // Run Opt2Swap and get the new sequence
                const auto new_sequence = opt2_swap(sequence, i, k);
                const auto new_fitness =
                    find_fitness_values(cost_matrix, new_sequence);
                if (new_fitness < best_fitness_value) {
                    I++;
                    sequence           = new_sequence;
                    best_fitness_value = new_fitness;
                }
            }
        }
        break;
    }
    std::cout << "Opt2 improvement cycles: " << I << std::endl;
    std::cout << "Fitness value of sequence after Opt2: " << best_fitness_value
              << std::endl;
}

/**
 * Load the VRP configuration parameters from the config file
 * @return TSP config parameters
 */
aco::Opt2Params aco::get_opt2_params() {
    Opt2Params params{};
    libconfig::Config cfg;
    try {
        const std::string package_name          = "aco_router";
        const std::string package_relative_path = "/config.cfg";
        const std::string filename =
            aco::get_directory_path(package_name, package_relative_path);
        char* tab2 = new char[filename.length() + 1];
        strcpy(tab2, filename.c_str());
        cfg.readFile(tab2);
    } catch (const libconfig::FileIOException& fioex) {
        std::__throw_invalid_argument("I/O error while reading file.");
    }

    try {
        params.max_iters = cfg.lookup("max_iters_opt2");
        params.use_opt2  = cfg.lookup("use_opt2");
    } catch (const libconfig::SettingNotFoundException& nfex) {
        std::cerr << "Missing setting in configuration file." << std::endl;
    }
    return params;
}

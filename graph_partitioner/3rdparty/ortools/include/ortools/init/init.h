#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "ortools/base/logging.h"
#include "ortools/gurobi/environment.h"
#include "ortools/sat/cp_model_solver.h"

ABSL_DECLARE_FLAG(std::string, cp_model_dump_prefix);
ABSL_DECLARE_FLAG(bool, cp_model_dump_models);
ABSL_DECLARE_FLAG(bool, cp_model_dump_lns);
ABSL_DECLARE_FLAG(bool, cp_model_dump_response);


namespace operations_research {

/** 
 * Simple structure that holds useful C++ flags to setup from non-C++ languages.
 */
struct CppFlags {
  /**
   * If true, all logging message will be sent to stderr.
   */
  bool logtostderr = false;

  /**
   * Controls is time and source code info are used to prefix logging messages.
   */
  bool log_prefix = false;

  /**
   * Prefix filename for all dumped files (models, solutions, lns sub-models).
   */
  std::string cp_model_dump_prefix;
  /**
   * DEBUG ONLY: Dump CP-SAT models during solve.
   *
   *  When set to true, SolveCpModel() will dump its model protos
   * (original model, presolved model, mapping model) in text  format to
   * 'FLAGS_cp_model_dump_prefix'{model|presolved_model|mapping_model}.pbtxt.
   */
  bool cp_model_dump_models = false;

  /**
   * DEBUG ONLY: Dump CP-SAT LNS models during solve.
   *
   * When set to true, solve will dump all lns models proto in text format to
   * 'FLAGS_cp_model_dump_prefix'lns_xxx.pbtxt.
   */
  bool cp_model_dump_lns;

  /**
   * DEBUG ONLY: Dump the CP-SAT final response found during solve.
   *
   * If true, the final response of each solve will be dumped to
   * 'FLAGS_cp_model_dump_prefix'response.pbtxt.
   */
  bool cp_model_dump_response;
};

/**
 * This class performs various C++ initialization.
 *
 * It is meant to be used once at the start of a program.
 */
class CppBridge {
 public:
   /**
    * Initialize the C++ logging layer.
    * 
    * This must be called once before any other library from OR-Tools are used.
    */
  static void InitLogging(const std::string& program_name) {
    google::InitGoogleLogging(program_name.c_str());
  }

  /**
   * Shutdown the C++ logging layer.
   * 
   * This can be called to shutdown the C++ logging layer from OR-Tools.
   * It should only be called once.
   */
  static void ShutdownLogging() {
    google::ShutdownGoogleLogging();
  }
  
  /**
   * Sets all the C++ flags contained in the CppFlags structure.
   */
  static void SetFlags(const CppFlags& flags)  {
    absl::SetFlag(&FLAGS_logtostderr, flags.logtostderr);
    absl::SetFlag(&FLAGS_log_prefix, flags.log_prefix);
    if (!flags.cp_model_dump_prefix.empty()) {
      absl::SetFlag(&FLAGS_cp_model_dump_prefix, flags.cp_model_dump_prefix);
    }
    absl::SetFlag(&FLAGS_cp_model_dump_models, flags.cp_model_dump_models);
    absl::SetFlag(&FLAGS_cp_model_dump_lns, flags.cp_model_dump_lns);
    absl::SetFlag(&FLAGS_cp_model_dump_response, flags.cp_model_dump_response);
  }

  /**
   * Load the gurobi shared library.
   *
   * This is necessary if the library is installed in a non canonical
   * directory, or if for any reason, it is not found.
   * You need to pass the full path, including the shared library file.
   * It returns true if the library was found and correctly loaded.
   */
   static bool LoadGurobiSharedLibrary(const std::string& full_library_path) {
     return LoadGurobiDynamicLibrary({full_library_path}).ok();
   }
};

}  // namespace operations_research
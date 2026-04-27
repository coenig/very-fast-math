import os
import time
import json
from itertools import product
from ctypes import CDLL, c_char_p, c_size_t

class Morty:
    def __init__(self):
        self.morty_lib = CDLL('lib/libvfm.so')
        self.morty_lib.expandScript.argtypes = [c_char_p, c_char_p, c_size_t]
        self.morty_lib.expandScript.restype = c_char_p
        self.morty_lib.morty.argtypes = [c_char_p, c_char_p, c_size_t]
        self.morty_lib.morty.restype = c_char_p

    def generate_smv_files(self, args: list[str]):
        self.morty_lib.generate_smv_files(";".join(args).encode('utf-8'))

def get_parameters_test_enumeration():
    # 007, NONEGOS -> non_egos
    # 008, NUMLANES -> lanes
    # 010, SECTIONS -> sections
    # SECTIONSMAXLENGTH -> max section length
    # SECTIONSMINLENGTH -> min section length
    # MODEL_INTERSECTION_GEOMETRY -> allow angles
    return {
      'nonegos':  [i for i in range(3, 6)],
      'sections': [i for i in range(3, 7)],
      'numlanes': [1],
      'section_max_length': [1000],
      'section_min_length': [100],
      'geometry': [1],
    }

def identity(x):
    return x


def get_parameters_simplest_geometries():
    return {
      'nonegos':  [5],
      'sections': [3],
      'numlanes': [1],
      'section_max_length': [1000],
      'section_min_length': [100],
      'geometry': [0],
    }

def get_param_simple(neg, sec, lanes, maxlen=1000, minlen=100, geom=1):
    return {
      'nonegos':  [neg],
      'sections': [sec],
      'numlanes': [lanes],
      'section_max_length': [maxlen],
      'section_min_length': [minlen],
      'geometry': [geom],
    }

def parametrize_model_config_with_dict(input_config, output_config, new_config):
    with open(input_config) as f:
        config = json.load(f)
        template = config["_config"]
        for k, v in new_config.items():
            template[k] = v

    json_output_config = json.dumps(config, indent=3)
    print(json_output_config)
    with open(output_config, 'w') as f:
        f.write(json_output_config)

def main():
    parameters = get_param_simple(
            neg=3, sec=2, lanes=1
    )
    # parameters = get_parameters_simplest_geometries()

    configs = [
      dict(zip(parameters.keys(), p))
      for p in product(*parameters.values())
    ]

    replace_config_key = {
      'nonegos': [("#007", identity), ("NONEGOS", identity)],
      'numlanes': [("#008", identity), ("NUMLANES", identity)],
      'sections': [("#010", identity), ("SECTIONS", identity)],
      'section_max_length': [("SECTIONSMAXLENGTH", identity)],
      'section_min_length': [("SECTIONSMINLENGTH", identity)],
      'geometry': [("MODEL_INTERSECTION_GEOMETRY", identity)],
    }

    all_configs = []
    for config in configs:
        current_dict = {}
        for k, multivalues in replace_config_key.items():
            for kk, vv in multivalues:
                current_dict[kk] = vv(config[k])
        all_configs.append(current_dict)

    input_config = 'main_template.json'
    experiments_dir = "SMV_GEN"
    try:
        os.mkdir(experiments_dir)
    except:
        pass

    all_configs.reverse()
    for i, config in enumerate(all_configs):
        output_config = f'/tmp/envmodel_config.json'
        parametrize_model_config_with_dict(input_config, output_config, config)

        ###
        args = [
            # json template
            output_config, # "/tmp/envmodel_config.tpl.json",
            # directory substring
            "0",
            # root dir
            ".",
            # envmodel tpl
            "src/templates/EnvModel.tpl",
            # planner path
            "src/examples/ego_less/vfm-includes.txt",
            # target directory
            f"SMV_GEN/scenario{int(time.time())}",
            # cached directory
            "examples/tmp",
            # template directory
            "src/templates",
        ]

        morty = Morty()
        morty.generate_smv_files(args)
        ###

        morty.generate_smv_files(f"{experiments_dir}/{i:03d}") # type: ignore


if __name__ == "__main__":
    main()

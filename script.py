import os
import time
import json
import subprocess
import networkx as nx
from pathlib import Path
from itertools import product
from pyvis.network import Network
import xml.etree.ElementTree as ET
from collections import defaultdict
from ctypes import CDLL, c_char_p, c_size_t

class NuXmvConn:
    process: subprocess.Popen
    property_map: dict[str, int]

    def __init__(self, nuxmv_args) -> None:
        self.process = subprocess.Popen(
                nuxmv_args,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
                )
        self.property_map = dict()

        # first prompt.
        _ = self._read_response()

        self._send("go_msat", ignore_output=True)

    def _read_response(self):
        """
        This method works under the assumption that there will, eventually be
        a respone from nuXmv's side. Usually used along with _send. It's blocking.
        """
        assert(self.process.stdout)
        output = ""
        while True:
            byte = self.process.stdout.read(1)
            if not byte:
                raise Exception(f"nuXmv internal error while reading response. Output captured: {output}")
            output += byte

            # nuXmv finished, since its showing the prompt
            if output.endswith("nuXmv > "):
                break
        return output[:-len("nuXmv > ")]

    def _send(self, messages: list | str, ignore_output=False):
        """
        Send a list of commands to nuXmv. It accepts either a list or a single
        command.
        """
        if not isinstance(messages, list):
            messages = [messages]
        elif not isinstance(messages, str):
            raise Exception("nuXmv Commands are in the form of strings!!!")

        # For pyright type checking.
        assert(self.process.stdin)
        assert(self.process.stdout)

        for msg in messages:
            msg += "\n"
            self.process.stdin.write(msg)
        self.process.stdin.flush()
        if ignore_output:
            _ = self._read_response()

class Morty:
    def __init__(self):
        self.morty_lib = CDLL('lib/libvfm.so')
        self.morty_lib.expandScript.argtypes = [c_char_p, c_char_p, c_size_t]
        self.morty_lib.expandScript.restype = c_char_p
        self.morty_lib.morty.argtypes = [c_char_p, c_char_p, c_size_t]
        self.morty_lib.morty.restype = c_char_p

    def generate_smv_files(self, args: list[str]):
        self.args = args
        self.morty_lib.generate_smv_files(";".join(args).encode('utf-8'))
        main_smv = Path(args[5]) / "main.smv"
        self.nuxmv = NuXmvConn([
            "nuXmv", "-quiet", "-int", main_smv
            ])

    def model_check(self, cex_file):
        self.nuxmv._send(
                f"msat_check_invar_bmc -i -a falsification -k 100",
                ignore_output=True
                )

        output_cex = Path(self.args[5]) / cex_file
        self.nuxmv._send(
                f"show_traces -p 6 -o {output_cex} 1",
                ignore_output=True
                )

        return output_cex

    def get_target_path(self):
        return Path(self.args[5])

    def block_topology(self, topology):
        # TODO 1: implement blocking the latest topology
        pass

class GraphBuilder:
    def __init__(self):
        pass

    def build_from_xml_cex(self, cex_file):
        tree = ET.parse(cex_file)
        root = tree.getroot()
        node = root.find('node')
        assert(node is not None)
        state = node.find('state')
        assert(state is not None)
        temp_dict = defaultdict(list)
        for child in state:
            variable = child.get('variable')
            if variable and "outgoing_connection" in variable:
                [_, _, start, _, _, end] = variable.split("_")
                temp_dict[f'S{int(start)}'].append(f'E{int(end)}')
                print(f'adding edge from {start} to {end}')

        return nx.DiGraph(temp_dict)

def dump_graph(graph, html_file='digraph.html'):

    net = Network(notebook=True, directed=True, cdn_resources='remote')

    net.from_nx(graph)
    net.show_buttons(filter_=['physics'])

    net.show(html_file)


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
                # 0: json template
                output_config, # "/tmp/envmodel_config.tpl.json",
                # 1: directory substring
                "0",
                # 2: root dir
                ".",
                # 3: envmodel tpl
                "src/templates/EnvModel.tpl",
                # 4: planner path
                "src/examples/ego_less/vfm-includes.txt",
                # 5: target directory
                f"SMV_GEN/scenario{int(time.time())}",
                # 6: cached directory
                "examples/tmp",
                # 7: template directory
                "src/templates",
                ]

        morty = Morty()
        morty.generate_smv_files(args)
        ###

        #  Main Loop.
        #  while True:
        #    1. model check.
        #    2. create graph from cex.
        #    3. if planar:
        #    4.   break
        #    5. Block topology of cex.

        #  Merge graph with the counter example, with 
        #  real units.

        iteration = 0
        gb = GraphBuilder()
        current_target_path = morty.get_target_path()
        while True:
            cex_file = morty.model_check(f'cex_it_{iteration}.xml')
            graph = gb.build_from_xml_cex(cex_file)
            dump_graph(
                graph,
                str(current_target_path / f'graph_it_{iteration}.html')
            )
            is_plannar, embedding = nx.check_planarity(graph)
            if is_plannar:
                concrete_graph = nx.combinatorial_embedding_to_pos(embedding)
                # TODO 2: show the final concrete graph
                break

            morty.block_topology(graph)

        # TODO 3: merge the stuff


def graph_builder_test():
    gb = GraphBuilder()
    graph = gb.build_from_xml_cex(Path('cex.xml'))
    dump_graph(graph, 'graph.html')

if __name__ == "__main__":
    graph_builder_test()
    # main()

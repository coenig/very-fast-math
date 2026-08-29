from __future__ import annotations

import subprocess
from ctypes import CDLL, c_char_p, c_size_t
from pathlib import Path

import networkx as nx


class NuXmvConn:
    def __init__(self, nuxmv_args: list[str]) -> None:
        self.process = subprocess.Popen(
            nuxmv_args,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True,
        )
        self.property_map: dict[str, int] = {}
        try:
            self._read_response()
        except BaseException:
            self.close()
            raise

    def _read_response(self) -> str:
        if self.process.stdout is None:
            raise RuntimeError("nuXmv stdout is not available")

        output = ""
        while True:
            character = self.process.stdout.read(1)
            if not character:
                raise RuntimeError(
                    "nuXmv stopped before returning a prompt. "
                    f"Output captured: {output}"
                )
            output += character
            if output.endswith("nuXmv > "):
                return output[:-len("nuXmv > ")]

    def _send(self, messages: list[str] | str, ignore_output: bool = False) -> None:
        if isinstance(messages, str):
            messages = [messages]
        elif not isinstance(messages, list):
            raise TypeError("nuXmv commands must be a string or list of strings")
        if self.process.stdin is None:
            raise RuntimeError("nuXmv stdin is not available")

        for message in messages:
            self.process.stdin.write(message + "\n")
        self.process.stdin.flush()
        if ignore_output:
            self._read_response()

    def close(self) -> None:
        if self.process.stdin:
            self.process.stdin.close()
        if self.process.stdout:
            self.process.stdout.close()
        self.process.terminate()
        self.process.wait()


class Morty:
    def __init__(self) -> None:
        self.morty_lib = CDLL("lib/libvfm.so")
        self.morty_lib.expandScript.argtypes = [c_char_p, c_char_p, c_size_t]
        self.morty_lib.expandScript.restype = c_char_p
        # self.morty_lib.morty.argtypes = [c_char_p, c_char_p, c_size_t]
        # self.morty_lib.morty.restype = c_char_p
        self.args: list[str] = []
        self.nuxmv: NuXmvConn | None = None

    def generate_smv_files(self, args: list[str]) -> None:
        self.args = args
        self.morty_lib.generate_smv_files(";".join(args).encode("utf-8"))
        target_path = Path(args[5])
        subprocess.run(
            [
                "external/linux64/kratos",
                "-trans_output_format=nuxmv-module",
                "-trans_enum_mode=symbolic",
                f"-output_file={target_path / 'planner.cpp_combined.k2.smv'}",
                str(target_path / "planner.cpp_combined.k2"),
            ],
            capture_output=True,
            text=True,
            check=False,
        )

    def model_check(
        self,
        cex_file: str,
        iteration: int,
        *,
        model_file: str | Path = "main.smv",
    ) -> Path | None:
        del iteration
        target_path = self.get_target_path()
        model_path = Path(model_file)
        if not model_path.is_absolute():
            model_path = target_path / model_path
        if not model_path.is_file():
            raise FileNotFoundError(f"SMV model does not exist: {model_path}")
        self.nuxmv = NuXmvConn(
            ["nuXmv", "-quiet", "-pre", "cpp", "-int", model_path]
        )
        self.nuxmv._send("go_msat")
        setup_output = self.nuxmv._read_response()
        print("SETUPTOUTPUT", setup_output)
        self._raise_on_nuxmv_error(setup_output, "go_msat")

        self.nuxmv._send("msat_check_invar_bmc -a falsification -k100 -n0")
        check_output = self.nuxmv._read_response()
        print("OUTP:", check_output)
        self._raise_on_nuxmv_error(
            check_output, "msat_check_invar_bmc"
        )
        normalized_output = check_output.lower()
        if (
            "counter-example" not in normalized_output
            and "counterexample" not in normalized_output
        ):
            print(f"No counterexample found for {model_path}")
            return None

        output_cex = target_path / cex_file
        output_cex.unlink(missing_ok=True)
        self.nuxmv._send(f"show_traces -p 6 -o {output_cex} 1")
        trace_output = self.nuxmv._read_response()
        print("OUTP:", trace_output)
        self._raise_on_nuxmv_error(trace_output, "show_traces")
        if not output_cex.is_file():
            raise RuntimeError(
                f"nuXmv reported a counterexample but did not create {output_cex}"
            )
        return output_cex

    @staticmethod
    def _raise_on_nuxmv_error(output: str, command: str) -> None:
        normalized_output = output.lower()
        error_markers = (
            "syntax error",
            "parser error",
            "cannot open",
            "must be flattened",
            "undefined symbol",
        )
        if any(marker in normalized_output for marker in error_markers):
            raise RuntimeError(f"nuXmv {command} failed:\n{output.strip()}")

    def close_model_checker(self) -> None:
        if self.nuxmv is not None:
            self.nuxmv.close()
            self.nuxmv = None

    def get_target_path(self) -> Path:
        if len(self.args) <= 5:
            raise RuntimeError("SMV files have not been generated")
        return Path(self.args[5])

    def block_topology(self, topology: nx.MultiDiGraph, iteration: int) -> None:
        del iteration
        edges_as_variables = []
        for start_node, end_node, data in topology.edges(data=True):
            if start_node == "NONE":
                minus_ones = data.get("data", [])
                for connection, section in minus_ones:
                    edges_as_variables.append(
                        f"env.outgoing_connection_{connection}"
                        f"_of_section_{section} = -1"
                    )
                continue

            connection = data.get("conn_number")
            if connection is None:
                continue
            start = str(start_node)[2:]
            end = str(end_node)[2:]
            edges_as_variables.append(
                f"env.outgoing_connection_{connection}_of_section_{start} = {end}"
            )

        if edges_as_variables:
            expression = " & ".join(edges_as_variables)
            with (self.get_target_path() / "main.smv").open("a") as main_smv:
                main_smv.write(f"INIT !( {expression} )\n")

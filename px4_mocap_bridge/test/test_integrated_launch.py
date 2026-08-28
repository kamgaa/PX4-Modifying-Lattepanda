import ast
from pathlib import Path


def _integrated_launch_tree():
    launch_file = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'v3_optitrack_flight_launcher.launch.py'
    )
    return ast.parse(launch_file.read_text(encoding='utf-8'))


def test_integrated_launch_includes_existing_launch_files():
    tree = _integrated_launch_tree()
    string_values = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    include_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'IncludeLaunchDescription'
    ]

    assert 'v3_flight_launcher.launch.py' in string_values
    assert 'optitrack_px4.launch.py' in string_values
    assert len(include_calls) == 2


def test_integrated_launch_forwards_optitrack_arguments():
    tree = _integrated_launch_tree()
    string_values = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }

    assert 'hostname' in string_values
    assert 'rigid_body_name' in string_values

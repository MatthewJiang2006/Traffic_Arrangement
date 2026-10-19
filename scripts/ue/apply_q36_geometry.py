"""Apply the saved Q36 mesh repair inside the UE editor, with a map backup.

Run with __file__ set to this script. No PIE or unrelated dirty maps allowed.
The shared SM_Road_9 asset is never edited; only this map's two instances change.
"""
import hashlib
import json
from pathlib import Path
import shutil
from datetime import datetime

import unreal

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'docs/native/q36_geometry'
MAP = '/Game/IndustrialArea/Map/IndustrialArea（复件）'
ORIGINAL = '/Game/IndustrialArea/Meshes/SM_Road_9.SM_Road_9'


def main():
    world = unreal.EditorLevelLibrary.get_editor_world()
    assert world.get_path_name().split('.')[0] == MAP
    assert unreal.EditorLevelLibrary.get_game_world() is None
    assert not unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages()
    manifest = json.loads((SOURCE / 'geometry_manifest.json').read_text(encoding='utf-8'))
    for name, digest in manifest['source_sha256'].items():
        assert hashlib.sha256((SOURCE / name).read_bytes()).hexdigest() == digest
    prefix = MAP + '.'
    actors = {a.get_name(): a for a in unreal.EditorLevelLibrary.get_all_level_actors()
              if a.get_path_name().startswith(prefix)}
    targets = {'SM_Road_55': 'SM_Road55_Q36Smooth',
               'SM_Road_87': 'SM_Road87_Q36Smooth',
               'TrafficInternalWaitingRamp_Q36_plate': 'SM_Q36_SmoothApron'}
    for name in targets:
        assert name in actors
    if all(actors[name].get_component_by_class(unreal.StaticMeshComponent).static_mesh.get_name()
           == mesh for name, mesh in targets.items()):
        unreal.log('Q36 geometry already applied; no changes made')
        return
    assert all(actors[name].get_component_by_class(unreal.StaticMeshComponent).static_mesh.get_path_name()
               == ORIGINAL for name in ('SM_Road_55', 'SM_Road_87'))
    assert all(name in actors for name in ('TrafficInternalWaitingRamp_Q36_-1',
                                          'TrafficInternalWaitingRamp_Q36_1'))
    disk_map = Path(unreal.Paths.project_content_dir()) / 'IndustrialArea/Map/IndustrialArea（复件）.umap'
    backup = Path(manifest['backup']).parent / ('Q36几何应用前_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    backup.mkdir()
    shutil.copy2(str(disk_map), str(backup / disk_map.name))
    materials = {
        'Inst_Asphalt_4': '/Game/IndustrialArea/Materials/Inst_Asphalt_4',
        'Mat_Concrete_10': '/Game/IndustrialArea/Materials/Mat_Concrete_10',
        'Mat_RoadLine_2': '/Game/IndustrialArea/Materials/Mat_RoadLine_2',
        'Mat_RoadLine_3': '/Game/IndustrialArea/Materials/Mat_RoadLine_3',
    }
    meshes = {}
    for asset_name in targets.values():
        asset_path = '/Game/TrafficRepairs/' + asset_name
        mesh = unreal.load_asset(asset_path)
        if mesh is None:
            options = unreal.FbxImportUI()
            options.import_mesh = True
            options.import_materials = options.import_textures = options.import_as_skeletal = False
            options.automated_import_should_detect_type = False
            options.mesh_type_to_import = unreal.FBXImportType.FBXIT_STATIC_MESH
            options.static_mesh_import_data.auto_generate_collision = False
            options.static_mesh_import_data.combine_meshes = True
            options.static_mesh_import_data.normal_import_method = unreal.FBXNormalImportMethod.FBXNIM_COMPUTE_NORMALS
            task = unreal.AssetImportTask()
            task.filename = str(SOURCE / (asset_name + '.obj'))
            task.destination_path = '/Game/TrafficRepairs'
            task.destination_name = asset_name
            task.automated = True
            task.options = options
            unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
            assert len(task.imported_object_paths) == 1
            mesh = unreal.load_asset(task.imported_object_paths[0])
        for index, slot in enumerate(mesh.static_materials):
            name = str(slot.material_slot_name)
            mesh.set_material(index, unreal.load_asset(materials[name]))
            unreal.EditorStaticMeshLibrary.enable_section_collision(
                mesh, not name.startswith('Mat_RoadLine'), 0, index)
        body = mesh.get_editor_property('body_setup')
        body.set_editor_property('collision_trace_flag', unreal.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE)
        body.set_editor_property('double_sided_geometry', True)
        assert unreal.EditorAssetLibrary.save_loaded_asset(mesh)
        meshes[asset_name] = mesh
    with unreal.ScopedEditorTransaction('Repair Q36 curb opening and continuous apron'):
        for name, asset_name in targets.items():
            actor = actors[name]
            actor.modify()
            assert actor.get_component_by_class(unreal.StaticMeshComponent).set_static_mesh(meshes[asset_name])
        plate = actors['TrafficInternalWaitingRamp_Q36_plate']
        plate.set_actor_scale3d(unreal.Vector(1, 1, 1))
        plate.set_actor_rotation(unreal.Rotator(0, 0, 0), False)
        plate.set_actor_location(unreal.Vector(-1800, 5700, 0), False, True)
        for name in ('TrafficInternalWaitingRamp_Q36_-1', 'TrafficInternalWaitingRamp_Q36_1'):
            assert unreal.EditorLevelLibrary.destroy_actor(actors[name])
        assert unreal.EditorLevelLibrary.save_current_level()
    unreal.log('Q36 geometry saved; regenerate map fingerprints and validate before reopening Q36')


main()

import os,subprocess,tempfile,json,pathlib
with tempfile.TemporaryDirectory(prefix='nav_guard_') as td:
    ps=pathlib.Path(td)/'ps'
    ps.write_text('#!/bin/sh\nprintf "%s\\n" "  2652 python3 /home/yang/yang_nav/install/nav_terrain/lib/nav_terrain/terrain_obstacles.py --ros-args"\n')
    ps.chmod(0o755)
    env=os.environ.copy();env['PATH']=td+':'+env['PATH'];env['OUT']=td+'/out'
    p=subprocess.run(['bash','tools/nav_regress/run_nav.sh'],cwd='/home/yang/yang_nav',env=env,text=True,capture_output=True,timeout=15)
    result={'returncode':p.returncode,'stdout':p.stdout,'stderr':p.stderr}
    assert p.returncode==3,result
    assert 'terrain_obstacles.py' in p.stdout,result
    print(json.dumps(result,indent=2))

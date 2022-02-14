#!/bin/bash

#compress wrl

name=tire_175-80r16

meshlabserver -i ${name}.wrl -o ${name}.obj  -s ../mesh_reduction.mlx -om fc
roseus "(load \"package://eus_assimp/euslisp/eus-assimp.l\")" \
         "(setq glv (load-mesh-file \"${name}.obj\" :scale 1.0))" \
         "(save-mesh-file \"${name}_exchanged.wrl\" glv :scale 1.0)"\
         "(exit)"

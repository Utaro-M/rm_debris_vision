### how to use
1. roslaunch rm_debris_vision recongnition_and_detection.launch (plane_extraction.launch) in vision PC
2. roslaunch display_plane_info.launch in field PC
3. roseus demo.l

### branches
realmachine: experiments for icra

### euslisp/
##### 卒論期＋アルファ
- class.l client/ demo.l functions-without-ri.l functions.l my-ik.l
##### タイヤ転がし整理後
- my-ik.l < my-ik-latest.l: opt ik関係
- class.l < params.l: 変数クラス
- ik-opt-paramインスタンスでoptmotiongen用の変数を管理する
  ik-opt-solverはそのインスタンスをうけとり軌道生成を行う

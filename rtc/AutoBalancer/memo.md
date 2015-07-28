### 座標系

##### 登場する座標系

- VRML-world座標
- rootLink座標
- 足の末端リンク座標
- 足のend-effector座標
- スタート時の両足end-coordsの真ん中座標
   - こんなもの存在しない？
   - fixLegToCoordsでこちらに変わる？

##### 具体例

ABCIKparamだと

- target_p0  target_r0
   - from : ~~スタート時の両足end-coordsの真ん中座標~~
   - from : VRML-world座標
   - to   : 足の末端リンク座標
- current_p0 current_r0
   - from : 
   - to   :
   - target_link->p / R を保持する
- localPos   localR
   - from : 足の末端リンク座標
   - to   : 足のend-effector座標
- target_end_coords
   - from : 
   - to   : 
- target_link->p target_link->R
   - from : VRML-world座標
   - to   : 足の末端リンク座標
- target_p0 / target_link->pはほぼ同じだが，前者はm_robotに非依存，後者はm_robotに依存なので，fixLegToCoordsに影響を受けない / 受けるという違いがある

leg_coords_generatorだと

- support_leg_coords
   - from :
   - to   :
-


fixLegToCoordsする前と後で変わりうるので注意


##### getCurrentParameters

- m_robotのrootLinkの位置姿勢を保持
- m_robotのangle-vectorを保持

##### getTargetParameters
この関数でやりたいことは

- 足以外のangle-vector指令を通す
- zmpのオフセットを補完する
- ikpのtarget_p0 / target_end_coordsを更新する
- m_contactStates / m_controlSwingsupportTime / m_limbCOPOffset を更新する
- ref_forces / sbp_ofset を表現する座標系を所望の座標系に変える
- target_root_p / target_root_R を決める
   - これのために，m_robot->rootLink()をずらしている？
- ref_cog / ref_zmp を決める


- m_robotのrootLinkの位置姿勢をshからもらったinput_baseに変える
   - これでVRMl-world座標が初期化される．前に歩いていても戻される．
- m_robotのangle-vectorをshからもらったm_qRefに変える
   - 以降target_link->pとtarget_p0は大きく違う．rootLink()が変わった＋angle-vectorが変わった．前者はfixLegToCoordsで戻されるが，後者は戻されないので，target_link->pは歩いていようが何していようがabcが入る前の足の姿勢になっていて，target_p0は歩いている姿勢．

以降はabcがonのときのみのコード

- gg->proc_one_tickでもろもろやる
- ikpのtarget_p0を新しいgg->get_support/swing_leg_coordsを使って更新する
- gg->get_swing_support_mid_coords(tmp_fix_coords)のtmp_fix_coordsが
   - from : スタート時の両足end-coordsの真ん中座標
   - to   : 現在の遊脚のdst/srcの間・支持脚のend-coordsの真ん中座標
- このtmp_fix_coordsでfixLegToCoordsが走る!!!
- target_root_p / target_root_R に m_robot->rootLink()->p / R を保存
- 腕のikpの targe_p0/r0をVRML-world座標のtarget_link->p/Rで上書きする :: ここは必要なのか？
- 足のikpの target_end_coordsを
   - from : スタート時の両足end-coordsの真ん中座標
   - to   : 現在のend-coords
- tmp_foot_mid_posは
   - from : スタート時の両足のend-coordsの真ん中座標
   - to   : angle-vectorがもとに戻った上での両足end-coordsの真ん中座標
- ref_cog / ref_zmp を設定
   - walking
      - ref_cog は x : preview_controller, y : preview_controller, z : calcCM
      - ref_zmp は x : preview_controller, y : preview_controller, z : preview_controller
   - stop
      - ref_cog は x : preview_controller, y : preview_controller, z : calcCM
      - ref_zmp は x : ref_cog, y : ref_cog, z : tmp_foot_mid_pos

以降はmode_sync_to_abcのときのみ

- current_root_p / R をtarget_p / R で上書きする
- 足のikp_target_p0 / r0 を target_link->p / R で上書きする


##### solveLimbIK
イメージとしては

- dif_cog = m_robot->calcCM() - ref_cog
- m_robot->rootLink()->p = m_robot->rootLink()->p + -0.8 * dif_cog

として目標の重心に追従させようと頑張ってる．

- 足のangle-vectorをもとに戻す
- root_link()をcurrent_root_p / R で上書きする
- dif_cogを計算して，rootLink()を更新する
- ik を解く


##### fixLegToCoordsがやっていること

両足のend-effectorの真ん中の座標系がfix_posとfix_rotで表される座標系になるようにm_robot->rootLink()を移動させる関数

- getTargetParameters
- stopWalking

の２箇所で使われている

current_foot_mid_pos : VRML-world座標系で表した両足のend-effectorの真ん中の位置
current_foot_mid_rot : VRML-world座標系で表した両足のend-effectorの真ん中の姿勢
fix_pos              : VRML-world座標系で表した目標の位置
fix_rot              : VRML-world座標系で表した目標の姿勢
tmpR                 : fix_rot * current_foot_mid_rot.transpose()
                     : これは裏に「current_foot_mid相対のBody座標＝fix_相対のBody座標」があって，tmpR自体に意味はない．





### biped only にしているところ

### 野沢さんに聞きたいこと

1. preview controller に swing zmp offset が入っているが，なぜか．N脚のときはどうすればいいか相談．
   - つま先だちするときに，zmp_offsetを使ってつま先の先端にCOPをもっていきたくて，そのために，preview_controlにzmp_offsetを渡したいから
   - done
1. getTargetParametersでtmp_fix_coordsのrotを修正しているのはなぜか．
   - これがないとfixLegToCoordsが動かない．腰を曲げたときにも動いているのはこいつのおかげ．コメントアウトして見てみると良い．
1. // m_ref_force frame : world frame ではなく // m_ref_force frame : current world frame な気がする．tmp_fix_coords.rotが現在の両足end-coordsの真ん中なので．
   - どちらでも良いから聞かなかった．
1. midcoordsの用途．goPosとかは目標位置を決める基準として使っているが，それ以外のmidcoordsの用途がよく分かっていなくて，そのためN脚にしたときにどう変更するのが正解か分かっていない．
   - 台車型の移動計画をしていて，ビヨーンとpathを描いてから，leg_offset文だけずらす，みたいなことをやっている．このとき，mid_coordsを使っている．基準座標系という名前にした方が良いかも．
   - leg_offsetを4きゃくの中心にするか，両足真ん中のままにするか，どちらがいいか pros /cons を上げてみると良い．
1. 腕がないパターンのロボットにも対応できているか確認が必要．
   - そこまでいっていないから聞かなかった
1. lcg.resetでswing_leg_dst_coordsとswing_leg_src_coordsの初期値を与えているが，proc_one_tickの中で呼ばれるlcg.update_leg_coordsではswing_leg_dst_coordsを上書きしている．初期値はどこで使われるの？
   - 最初の一歩で使われている．

---

1. inside step limitationでやりたいことの左右が反転してる？
   - 自己解決．反転していなくて，合っている．なぜならsnは支持脚で，動かすのは逆足だから．

### TODO



### メモ

÷2とかしているところはleg_names.size()に置き換えればいい．
mid_rotのようにアルゴリズム的に2個を想定しているのは悩みどころ

### 興味

1. vel_htcは何でしょう．



### できていること

- append_go_pos_step_nodes (const coordinates& _ref_coords, const std::vector<leg_type>& lts) で複数の足をfoot_stepnodes_listに入れることができるようになった．

### できていないこと


### 構想
- goPos的な何かでcrawl歩行か何かが出来れば良さそう
   - goPosをローカルで改造して4足歩行のikを解き始める段階まで行くのが最初のステップ

- AutoBalancer::goPos(const double& x, const double& y, const double& th)
   - 入力は一般性あり

#### 野沢さんに聞きたいこ2

- initialize_gait_parameterの最初の方で，一歩目を上書きしているのはなぜ？
- printしたらかわっていないみたい

> 1. lcg.resetでswing_leg_dst_coordsとswing_leg_src_coordsの初期値を与えているが，proc_one_tickの中で呼ばれるlcg.update_leg_coordsではswing_leg_dst_coordsを上書きしている．初期値はどこで使われるの？
   - 最初の一歩で使われている．

とのことだったけども，上のことと関連して

- go_pos_param_2_footstep xxx で footstep_nodes_listを決めて
- initialize_gait_parameter で footstep_nodes_listの一歩目を上書きして
- その中のlcg_set_swings_supports_listで1つ前support_legs_coords_listの一番最初の値を今回のsupport_legs_coords_listにいれている

が，これはどういうあれか

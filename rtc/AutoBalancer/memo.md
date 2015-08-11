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


### 興味

1. vel_htcは何でしょう．



### できていること

- append_go_pos_step_nodes (const coordinates& _ref_coords, const std::vector<leg_type>& lts) で複数の足をfoot_stepnodes_listに入れることができるようになった．

### できていないこと

- "leg"が4つくらいある
   - 1つ目 /  4つ目はつま先用の補正なので，あって良い
   - 2つ目は腕特別扱いパターンで，よく分からないから考える
      - startABCで腕もis_activeになっている場合，このあとsolveLimbIKするので，目標値がこの値にセットされるため，歩いていても手先が絶対座標系で固定される．
      - it->second.target_p0 = it->second.target_link->p;
      - ちなみに，脚のtarget_p0のみ変更 -> fixLeg -> 今回の腕のみの部分，という流れ
      - N客にするには一番最初の部分で腕も変更して，今回の部分をなくす必要がある．
      - 前者はできているので，後者を変える．
      - done!!!
   - 3つ目は足特別扱いパターンで，よくわからないから考える
      - MODE_SYNC_TO_ABCの場合のみ
      - it->second.target_p0 = it->second.target_link->p;

### 構想
- goPos的な何かでcrawl歩行か何かが出来れば良さそう
   - goPosをローカルで改造して4足歩行のikを解き始める段階まで行くのが最初のステップ

- AutoBalancer::goPos(const double& x, const double& y, const double& th)
   - 入力は一般性あり

#### 野沢さんに聞きたいこと2

- initialize_gait_parameterの最初の方で，一歩目を上書きしているのはなぜ？
- printしたらかわっていないみたい

> 1. lcg.resetでswing_leg_dst_coordsとswing_leg_src_coordsの初期値を与えているが，proc_one_tickの中で呼ばれるlcg.update_leg_coordsではswing_leg_dst_coordsを上書きしている．初期値はどこで使われるの？
   - 最初の一歩で使われている．

とのことだったけども，上のことと関連して

- go_pos_param_2_footstep xxx で footstep_nodes_listを決めて
- initialize_gait_parameter で footstep_nodes_listの一歩目を上書きして
- その中のlcg_set_swings_supports_listで1つ前support_legs_coords_listの一番最初の値を今回のsupport_legs_coords_listにいれている

が，これはどういうあれか

- get_swing_support_mid_coordsがbiped onlyだった．これはfixLegToCoordsとも関係している．
   - 登場人物は
      - fixLegToCoords
      - fix_leg_coords : abcのinitializeの段階で定義されて．その後ずっと使われる．
      - tmp_fix_coords : getTragetParameterで毎回定義されなおす．
   - くらい

   - gg_is_walking のときは
      - gg->get_swing_support_mid_coords(tmp_fix_coords);
   - !gg_is_walking のときは
      - tmp_fix_coords = fix_leg_coords;
   - そのあとに !adjust_footstep_interpolator->isEmpty() なら
      - fix_leg_coords = いい感じ(adjustなんちゃらを考慮した)に計算した両足end-coordsの真ん中
      - tmp_fix_coords = fix_leg_coords;
   - さらにそのあとに
      - tmp_fix_coordsを水平にする
   - で，fixLegToCoords(tmp_fix_coords.pos, tmp_fix_coords.rot);する

   - stopWalkingのときに
      - fix_leg_coords = 両足のend-coordsの真ん中

- N脚のときにもmid_coordsをできるようにするのはできそう -> できた
- あとは，FixLegToCoordsが微妙っぽい
- getCurrentParamters 直後は target_p0 / target_link->p ともにいい感じで，
- getTargetParamters 直後に ずれて，solveLimbの中ではちょっと戻る．変化するのはtarget_link->pの方なので，fixLegToCoordsが影響しているはず
- fixLegToCoordsを手足の真ん中にすれば良い？
- fixLegToCoordsをそうしちゃうと，leg_posが足裏基準なのでずれちゃう．．．
   - 関係無さそうで，問題は違うところにあるっぽい

- もう一度座標系を整理する必要あり 絵を書く？
- 登場人物は
   - fix_leg_coords
   - tmp_fix_coords
   - rootLink->p / R
   - target_end_coords
   - swing_legs_src_coords / swing_legs_dst_coords / support_legs_coords
   - footstep_nodes_list
- くらい

- fixLegToCoordsしているのは，eusから与えたangle-vectorのときの腰の位置姿勢をabcの中でのm_robotの腰の位置姿勢に与えたいからで．


### 歩かない場合
1. eus から angle-vector(例えばreset-pose) を 送る．rootLinkの位置姿勢とかは送らずに純粋にangle-vectorだけを送る．
   - onExecuteの中でgetTargetParamtersが呼ばれる
      - 姿勢がangle-vectorになりつつ，rootLinkは宙に浮くようになる（VRMLのwaistの位置姿勢にセットされる）
      - tmp_fix_coords = fix_leg_coords(最初は原点)
      - getTargetParametersの中のfixLegToCoordsにて，両足の真ん中がtmp_fix_coordsに一致するよう，全体的に動く
      - target_end_coordsが計算されて保存される

### start-auto-balancerした場合
1. 上の一連の流れが終わっていたとして，eus から start-auto-balancerした
   - abc の startAutoBalancer -> startABCParam で is_activeがセットされる
   - mode_sync_to_abcの間にtarget_p0 / target_r0 が現在地にセットされる
   - で，onExecuteが呼ばれてgetTargetParametersが呼ばれる
      - tmp_fix_coords = fix_leg_coords(最初は原点)
      - getTargetParametersの中のfixLegToCoordsにて，両足の真ん中がtmp_fix_coordsに一致するよう，全体的に動く
      - target_end_coordsが計算されて保存される
      - ref_cogが両足のend-coordsの真ん中にセットされる．zだけ別枠で現在の重心位置 related to WORLD になる．
   - solveLimbIKが呼ばれる
      - rootLinkにcurret_root_p/Rがセットされる
      - rootLinkの位置が現在の重心とref_cogの差分だけ動く．姿勢は変わらない．
      - で，target_p0 / target_r0 に向けて ik を解く

### rootLink / angle-vector 関連
1. getCurrentParameters
   - current_root = m_robot->rootLink()
   * qorg = m_robot->joint
1. getTargetParameters の最初
   - m_robot->rootLink() = seqから来た指令値
   * m_robot->joint = seqから来た指令値
1. getTargetParameters の真ん中
   * 足のみ： target_p0 / r0 = footstepから計算
      - これはもっと後ろでいいのでは．後ろというか腕のtarget_p0付近でいいのでは．
   - fixLegToCoords(tmp_fix_coords.pos, tmp_fix_coords.rot);
   - A: 現在のrootLink / angle-vector から計算した両足のend-coords
      - seqから来た値が入っていることがポイント
   - B: swg_coords と sup_coords の真ん中くらい
      - tmp_fix_coords : swing_support_mid_coords
   - A が B に一致するようm_robot->rootLinkを動かす
1. getTargetParameters の最後
   - target_root = m_robot->rootLink()
      - ここで rootLink の高さをtarget_rootに教えることができる
   * 腕のみ： target_p0 / r0 = target_link->p / R
   * 足のみ： target_end_coords = m_robot の end_coords
   - tmp_foot_mid_pos *= (1.0 / leg_names.size());
      - 歩いていないときはここでref_cogが決まる．
      - 座標系はfixLegToCoords後の座標系
1. getTargetParameters の最後 IF MODE_SYNC_TO_ABC
   - current_root = target_root
   * 足のみ： target_p0 / r0 = target_link->p / R
1. solveLimbIK
   * 足のみ： m_robot->joint = qorg
   - m_robot->rootLink() = current_root
      - x / y のため？
   - dif_cog(2) = m_robot->rootLink()->p(2) - target_root_p(2);
   - m_robot->rootLink()->p = m_robot->rootLink()->p + -1 * move_base_gain * dif_cog;
   - m_robot->rootLink()->R = target_root_R;
   * is_active のみ： target_p0 / r0 に向かってIKを解く



### 歩く場合
reset-poseを送っていて，上の一連の流れが終わっているとする．
1. eus から go-pos 0 0 0 を送る
   - abcのgoPosが呼ばれ，start_ref_coordsが両足のtarget_end_coordsの真ん中になる
   - これをstart_ref_coordsにする（ので，この場合は原点）
   - これからfootstep_nodes_listを計算していく
   - で，onExecuteが呼ばれてgetTargetParametersが呼ばれる
   - 姿勢がangle-vectorになりつつ，rootLinkは宙に浮くようになる（VRMLのwaistの位置姿勢にセットされる）
   - 



goPos

#### goPosTrotすると暴れる
go-posのときと比較すると，target_p0はいい感じだけど，target_link->pが全然ダメで．腕のupperlimitにかかっている．


name : larm
target_p0 :     [-0.0195049,  -0.0900176,  -0.183879] <!-- rlegの目標値になっている -->
target_link->p :     [0.0184685,  0.312079,  0.437051]

name : lleg
target_p0 :     [-0.0195485,  0.0899961,  -0.233873]
target_link->p :     [-0.0195641,  0.0894368,  -0.233871]

name : rarm
target_p0 :     [0.0195546,  -0.322973,  0.423866]
target_link->p :     [0.0196154,  -0.323662,  0.423769]

name : rleg
target_p0 :     [0.0194988,  0.322952,  0.373886] <!-- larmの目標値担っている -->
target_link->p :     [-0.0195125,  -0.0905632,  -0.233876]


- 目標値が腕と足でひっくり返っている
   - sortが原因か
   - 違うみたい
- 登場人物は
   - swing
      - swing_legs_dst_coord
         - fnslの順番
      - swing_legs_src_coords
         - swing_legs_dst_coords_list / support_legs_coords_list 順
      - swing_legs_dst_coords_list
         - set_swings_supports_list でセットされる
      - swing_legs_coords
         - calc_current_swing_legs_coods 順
      - get_swing_legs
         - lcg.get_swing_legsの返り値をconvert_leg_types_to_stringsした順
   - support
      - support_legs
         - get_support_leg_types_from_footstep_nodes順
      - support_legs_coords
         - support_legs_coords_list順
      - support_legs_coords_list
         - set_swings_supports_list でセットされる

- swg / sup ともに揃う理由がないので，明示的にクラス変数として保持して，こちらで揃えてあげる必要がありそう @ update_legs_coords
- support_legs_list 的なのをセットする @ set_swings_supports_list
- 四肢の名前情報を持っているのはstep_nodeなので，そこから引き出す．

- それには命名を整理する必要有り
   - 変数
      - swing_legs
         - ないので作る
      - foot_x_axises_list
         - refzmp_generator
      - swing_legs_list
         - refzmp_generator
      - support_legs
         - leg_coords_generator
      - all_limbs
         - gait_generator
      - swing_legs_dst_coords_list
         - leg_coords_generator
      - support_legs_coords_list
         - leg_coords_generator
      - support_legs_coords
         - leg_coords_generator
      - swing_legs_coords
         - leg_coords_generator
      - swing_legs_src_coords
         - leg_coords_generator
      - swing_legs_dst_coords
         - leg_coords_generator
   - 関数
      - get_swing_legs
      - get_support_legs

#### memo

1. 支持脚・ゆう客を交互にする前提になっているので，goPosCrawlはできない．Trotとかならできる．

1. leg_namesを外から変えられるようにする
    - 野沢さんのコメントのようにパラメータとしてできるようにする

#### ~~coordinatesに名前をつけて，RLEG / LLEG / RARM / LARM を判定する作戦~~ -> step_nodeを使う作戦に変更

~~つまるところ，FNS_listを作る際のstep_node1つ1つのcoordsに名前をつけてあげれば，後から取り出すのもここoriginなので，勝手にcoordsに名前がついてくれて簡単になりそう~~
-> footstep_nodes_list はこのままでOK, swing_legs_coordsとかの部分をswing_leg_nodesとかにrenameしつつ，中身を変えていく感じ

- goPos
  - go_pos_param_2_footstep_nodes_list : これは本当にgoPos用・setFootStepでは出てこない
    - start_ref_coords / goal_ref_coordsを計算する
    - 1歩目はcoordinatesとその名前を引数で与えているからOK
    - append_footstep_list_velocity_mode を for で回す
      - ref_coords と fns_list.back()をappend_go_pos_step_nodesに与える
        - FNS_list.back と all_limbs の差分をswing_namesとして求めて，それとref_coordsを用いて，append_go_pos_step_nodesを呼ぶ
          - append_go_pos_nodesではswing_namesの順番で，ref_coordsとleg_default_translation_posを用いて，FNSを計算していく
            - つまり，append_go_pos_nodes はOKという意味
          - 前回のswingの逆足が今回のswingとなっていて，これだとcrawlできないので，ダメ
            - crawlはsetFootStepでやると考えると一旦ここは放置で良さそう
            - 遊脚起動生成がcycloid shuffle などあるように，支持きゃくからゆうきゃくを求める関数が会ってもいいのかも
    - append_go_pos_nodesでもろもろ追加していくOK
    - append_finalize_footstepで最後のFNSを追加
      - 最後のFNSには2つ前のFNSを入れているので，これも問題ない
    - まとめるとgo_pos_param_2_footstep_nodes_listで作られるfootstep_nodes_listの1つ1つのnodeのl_rとcoordinatesは対応しているから安全
  - startWalking : こちらはsetFootStepででてくるので，1歩前のゆうきゃくが今の支持脚理論は成り立たないからやっかい
    - 

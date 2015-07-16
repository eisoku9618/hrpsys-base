### 座標系

登場人物は

- VRML相対
- rootLink相対
   - ikp.target_link
- 足リンク相対
   - ikp.localPos
- end effector 相対
   - default_zmp_pffsets


VRML相対というのは，world座標相対とは違う．

例えば，

```lisp
(send *robot* :reset-pose)
(send *robot* :legs :move-end-pos #f(0 0 400) :local)
(send *ri* :angle-vector (send *robot* :angle-vector) 1000)
```

しても，VRML相対の*input_basePos*は不変．

つまり，VRML相対は，VRMLファイルの原点だと思われる．



---

```bash
rtprint localhost:15005/abc.rtc:basePosOut
```

しつつ，

```lisp
(send *robot* :reset-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 1000)
(send *ri* :wait-interpolation)
(walking-pose *robot* :root-link-pitch-offset 15 :chest-link-pitch-offset 20 :root-link-height-offset -150 :default-pose-method :reset-pose)
(send *ri* :wait-interpolation)
```

すると値は変わっているので，どこらへんでrootlinkの位置・姿勢が変わるのか要確認．

このとき

```bash
rtprint localhost:15005/sh.rtc:basePosOut
```

は不変なので，*input_basePos*は不変のはず．


-> rtprint localhost:15005/abc.rtc:basePosOut vs rtprint localhost:15005/sh.rtc:basePosOut でちょっと違っていて，abcの中でfixLegToCoordsしているから，後者は不変だけど前者は変わる


---

ikp の座標系

- ヒント
   - tmp_foot_mid_pos += tmpikp.target_link->p + tmpikp.target_link->R * tmpikp.localPos + tmpikp.target_link->R * tmpikp.localR * default_zmp_offsets[i];
   - tmprc.pos = ikp["rleg"].target_p0 + ikp["rleg"].target_r0 * ikp["rleg"].localPos;

- target_r0 : rootLink相対の目標点の姿勢
- current_r0 :
- localR : 足先リンク相対のend-effectorの姿勢
- adjust_interpolation_target_r0
- adjust_interpolation_org_r0

- target_link : rootLink相対の足先リンク

---

fix_coords / mid_coords の謎

```cpp
hrp::Matrix33 eeR;        /* rootLink相対のend-effectorの姿勢 */
...
// m_ref_force frame : End effector frame
//ref_forces[i] = eeR * hrp::Vector3(m_ref_force[i].data[0], m_ref_force[i].data[1], m_ref_force[i].data[2]);
// m_ref_force frame : world frame
ref_forces[i] = tmp_fix_coords.rot * hrp::Vector3(m_ref_force[i].data[0], m_ref_force[i].data[1], m_ref_force[i].data[2]);
```

を見るに，m_ref_force はrootlink相対の力で，tmp_fix_coords.rotはrootLink相対のworld座標の回転を表している．

tmp_fix_coordsは

- 歩いているとき：gg->get_swing_support_mid_coords -> 遊脚と支持脚の間 -> スタート地点での両足end-coordsの真ん中相対
- 止まっているとき：fix_leg_coords -> 下と大体一緒？
- adjust_footstep_interpolatorのとき：スタート地点での両足end-coordsの真ん中相対のend-effectorの両足の真ん中

なので，tmp_fix_coordsをworld座標だと想定しているっていうこと？

または，ref_forcesをworld frameと書いてあるのはミスリードで，ref_forcesはend-effectorの真ん中の座標系を基準にして入力する，っていうこと？

tmp_fix_coordsをprintしてみたら，なんか違う？
腰を曲げた状態で，
.pos : 0 0 0
.rot : E

で，go-pos 1 0 0で歩かせたら

.pos : 1 0 0
.rot : E

だった．

go-pos 0 0 30やったらrotもそうなった

gg->get_support_leg_coords()を表示したら，world相対とかrootLink相対ではなくて，以下のようにスタート時の足裏真ん中座標系相対だった．

support leg
[1.5,  0.1,  0]
[1, 0, 0]
[0, 1, 0]
[0, 0, 1]

ikp[rleg].target_p0:
1.5
-0.1
0.096

ikp[rleg].target_r0 * ikp[rleg].localPos
0
0
-0.096

tmprc.pos
1.5
-0.1
0

- target_p0 : スタート地点での両足end-coordsの真ん中相対の現在の足先リンク座標の位置
- localPos  : 足先リンク座標相対のend-coords座標の位置
- swing


1.5mm 歩いた後に， 250 0 150のfootstepで階段を登る

support leg
[1.75,  0.1,  0.15]
[1, 0, 0]
[0, 1, 0]
[0, 0, 1]

ikp[rleg].target_p0:
1.75
-0.1
0.246

ikp[rleg].target_r0 * ikp[rleg].localPos
6.70631e-17
1.05063e-10
-0.096

tmprc.pos
1.75
-0.1
0.15


1.5mm 歩いた後に， 250 0 150 / -45deg :y のfootstepで階段を登る

support leg
[1.75,  0.1,  0.15]
[    0.707107, -5.40382e-25,    -0.707107]
[ 2.23833e-25,            1, -5.40382e-25]
[    0.707107,  2.23833e-25,     0.707107]

ikp[rleg].target_p0:
1.68212
-0.1
0.217882

ikp[rleg].target_r0 * ikp[rleg].localPos
0.0678823
2.44817e-10
-0.0678823

tmprc.pos
1.75
-0.1
0.15


スタート地点での両足end-coordsの真ん中を保持している何かがあるはず

target_link->pは何者？

tmprc.pos
1.75
-0.1
0.15

ikp[rleg].target_link->p:
1.75
-0.1
0.246

のようにzだけ違う．
普通に，スタート地点での両足end-coordsの真ん中相対のリンクの位置？回転が含まれてないから違う？

### biped only

1. xxx
1. yyy


### メモ

÷2とかしているところはleg_names.size()に置き換えればいい．
mid_rotのようにアルゴリズム的に2個を想定しているのは悩みどころ

### 不明点

##### for quad

1. preview controller に swing zmp offset が入っているが，なにこれ．
1. midcoordsの用途．goPosとかは目標位置を決める基準として使っているが，それ以外のmidcoordsの用途がよく分かっていなくて，そのためN脚にしたときにどう変更するのが正解か分かっていない．
1. 腕がないパターンのロボットにも対応できているか確認が必要．
1. vel_htcが謎．これはどう扱えばいいのか確認
1. VRML原点相対 or スタート時の両足end-coordsの真ん中相対
   - ２つある．前者はrootLink->pとかtarget_link->p，後者はtmp_fix_coordsとか．
1. fixLegToCoordsの回転行列のかけ方逆？
   - hrp::Matrix33 tmpR (fix_rot * current_foot_mid_rot.transpose()); ではなくて
   - hrp::Matrix33 tmpR (current_foot_mid_rot.transpose() * fix_rot);
   - 確かめるには fix_rotもcurrent_foot_mid_rotも単位行列ではなくすればOK
   - fix_xxx : 足座標でのx軸単位ベクトルをworld相対で見て，かつ，z方向を0にして，正規化したもの
   - current_foot_mid_xxx : VRML-world相対のend-effectorの真ん中
1. lcg.resetでswing_leg_dst_coordsとswing_leg_src_coordsの初期値を与えているが，proc_one_tickの中で呼ばれるlcg.update_leg_coordsではswing_leg_dst_coordsを上書きしている．初期値はどこで使われるの？

##### 興味
1. current_root_p / target_root_p とかのrootLinkのpの取り扱い方がよくわからない．solveLimbIKでは何が起こっている？


target_link->pとかは何があろうと変わらない？
target_link->pとかのm_robotが持っているのは，ほぼほぼ*robot*みたいなもので，*ri*がその場旋回しようが，階段を登ろうが変わらない，
ただ，move-end-posして足を上げて*ri*に送ったりすると変わる



tmpikp.target_link->p :
[-3.21965e-17,  -0.1,  0.141834]

tmpikp.target_link->R :
[          1,           0, 1.11022e-16]
[          0,           1,           0]
[-1.11022e-16,           0,           1]

tmpikp.localPos :
[0,  0,  -0.096]

tmpikp.localR :
[1, 0, 0]
[0, 1, 0]
[0, 0, 1]

tmpikp.target_link->p :
[-3.21965e-17,  0.1,  0.141834]

tmpikp.target_link->R :
[          1,           0, 1.11022e-16]
[          0,           1,           0]
[-1.11022e-16,           0,           1]

tmpikp.localPos :
[0,  0,  -0.096]

tmpikp.localR :
[1, 0, 0]
[0, 1, 0]
[0, 0, 1]

fix_pos :
[0.1,  2.77556e-17,  0.2]

fix_rot :
[1, 0, 0]
[0, 1, 0]
[0, 0, 1]

current_foot_mid_pos :
[-4.28546e-17,  0,  0.0458336]

current_foot_mid_rot :
[          1,           0, 1.11022e-16]
[          0,           1,           0]
[-1.11022e-16,           0,           1]

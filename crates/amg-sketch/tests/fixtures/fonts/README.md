# Test fonts (SK-10)

`NotoSans-{Regular,Bold,Italic}-ascii.ttf` are subsets (U+0020..U+007E, no hinting, no layout
features; made with `fontTools.subset`) of Noto Sans from
<https://github.com/notofonts/notofonts.github.io>, copyright The Noto Project Authors,
licensed under the SIL Open Font License 1.1 (`OFL.txt`). They are plain files, not git-LFS,
so the CI test job (which does not fetch LFS objects) has them. The Reserved Font Name rule of
the OFL applies to distributing *modified* fonts under the same name; these subsets are test
data only and are never shipped.

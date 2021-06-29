# Aortic 3D Deformation Reconstruction

This project is a learning material for 2021 Winter school on SLAM in deformable environments. Here, we provide a Matlab implementation of embedded deformation graph [1]. This implementation is used for the tutorial class. We also provide some functions of a Matlab implementation of our ICRA 2020 paper related to aortic deformation reconstruction [2]. This is used for the project assignment. The whole code will be released after the winter school. 

## EDGraph

The implementation of EDGraph is in `EDGraph`. You can run `./EDGraph/main_edgraph3D.m` using Matlab, and see how the ear of a bunny is deformed according to the given control points. The slides of the tutorial will also be uploaded in `document` soon. 

## Aortic 3D Reconstruction

The Matlab functions for aortic 3D deformation reconstruction is in `AortaDef`. A brief introduction of this project assignment is `./document/WinterSchool_Project_AorticDef_intro.pdf`. You can refer to `./document/WinterSchool_Project_AorticDef_doc.pdf` for more details  on how to finish this assignment.



## Contacts:

Yanhao Zhang: yanhao.zhang@student.uts.edu.au. Any discussions or concerns are welcomed :)

## Citation

Embedded deformation graph:

```
@incollection{sumner2007embedded,
  title={Embedded deformation for shape manipulation},
  author={Sumner, Robert W and Schmid, Johannes and Pauly, Mark},
  booktitle={ACM SIGGRAPH 2007 papers},
  pages={80--es},
  year={2007}
}
```

Aortic 3D deformation reconstruction:

```
@inproceedings{zhang2020aortic,
  title={Aortic 3D deformation reconstruction using 2D x-ray fluoroscopy and 3D pre-operative data for endovascular interventions},
  author={Zhang, Yanhao and Zhao, Liang and Huang, Shoudong},
  booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={2393--2399},
  year={2020},
  organization={IEEE}
}
```
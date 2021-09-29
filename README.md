# SmartSurvey_QGC

**QGroundControl**

This is an extension of QGroundControl that provides some additional features in QGControl to optimize the drone trajectory and energy consumption.

The list of additional features, their description and their use are mentioned on the [link](https://docs.google.com/presentation/d/1INAbxRJxul3wHkVudp6JSQHbo7yK59PzovAylaD3nAE/edit?usp=sharing).


**Installation and Pre-requisites**

Please refer to QgroundControl [Github page](https://github.com/mavlink/qgroundcontrol) for installation instructions


**How to Run**

The files that are changed for adding these features are _SurveyComplexItem.cc_, _SurveyComplexItem.h_, _Survey.SettingsGroup.json_ in the **MissionManager** folder under src and the _SurveyItemEditor.qml_ file in **PlanView** folder under src. Compile the code on Qt after replacing these four files.


**References**
[1] http://qgroundcontrol.com/

[2] S. Iyengar, R.R. Saxena, J. Pal, B. Chhaglani, A. Ghosh, V.N. Padmanabhan, P.T. Venkata 2021. Holistic Energy Awareness for Intelligent Drones In Proceedings of ACM International Conference on Systems for Energy-Efficient Buildings, Cities, and Transportation (BuildSys ’21), November 17–18, 2021, Coimbra, Portugal.ACM, New York, NY, USA, 11 pages. https://doi.org/10.1145/3486611.3486651


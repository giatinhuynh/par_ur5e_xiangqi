2 Assessment Details
The purpose of the project is to enable a robot to autonomously complete a task. Each project is scoped
differently and the requirements of each project are laid out in Section 4. All of the projects share the features
described below, which you will be required to address or conduct through the course of your project.
2.1 Investigation and Research
You will need to investigate and research algorithms/techniques related to your project. You will need to
implement and adapt these existing techniques to your particular needs. In your report, you must record the
findings of your investigate, and cite research relevant to your work. You must fully cite any references that
you have used within your research, implementation, analysis and/or evaluation.
2.2 Original Implementation
You must complete an original implementationof at least one significant element of the autonomous software
for the robot as part of your project. That is, your work cannot exist entirely of off-the-shelf software, such as
existing ROS2 packages. The scope of this original implementation is defined within the project definition and
negotiation with the course coordinator. This original implementation may:
• Significantly modify or adapt existing techniques and algorithms for practical use on a robot.
• Be a novel implementation of your own creation.
Your report should describe your original implementation.
2.3 Robot Software Architecture
You must consider and choose an appropriate robot software architecture (such as SPA, Subsumption, or a
three-tier architecture) for implementing your solution to the project. You will need to describe and justify
yourchoiceofrobotsoftwarearchitectureinyourreport, andshowthatyourprojectadherestothisarchitecture.
2.4 Autonomy
Your software should be fully autonomous, unless the scope of your project says otherwise.
2.5 Open-ended Exploration
Eachprojectisdesignedtobeopen-ended, allowingformultipleavenuesofexploration. Apurposeoftheproject
is for students to synthesis their knowledge across the course. More complex solutions to the projects displaying
more complex autonomous software will receive higher grades compared to trivial or simplified approaches.
2.6 Demonstration & Analysis
You (all groups) must complete a demonstration your autonomous software at completing the challenges of your
particular project. You (all groups) must also complete an analysis of their software, presented in your report.
This analysis should:
• Highlight the strengths and capabilities of your work.
• Identify limitations or weakness of your work.
3
You will need to collect evidence to support what the capability of your project.
2.7 Report
You must write a report to accompany your final demonstration. Details are provided in Section 6.

3 Separate PG and UG requirements
The requirements for PG and UG students, to satisfy these different levels of study are described in this section.
Please review this for your level of study.
3.1 ROS2 Infrastructure & Robot Software Architecture (UG)
For working with some robot platforms, the software infrastructure is not prepared to the same degree as with
the ROSBot’s. Some platforms may also use older ROS2 systems, such as Humble. This is often due to the
robots being setup for research purposes. Therefore, some projects will require UG students to implementation
additional infrastructure to enable working with the platform.
This may also include infrastructure that is implemented to ensure that the project is implemented according
to the chosen robot software architecture. This can include implementing data structures and ROS2 nodes that
enforce the flow of sensing information and behaviour generation.
3.2 Extended/Additional Algorithm Implementation (UG)
All projects have different algorithms/approaches that can be used to complete each project. UG students
will be expected to implement multiple solutions to their project, if the infrastructure implementation is not
sufficiently significant (or were not attempted). Unless otherwise noted in the project description, this will be
discussed with the course coordinator on a per-project basis.
3.3 Experimental Design and Evaluation (PG)
As studies extensively in this course, every algorithm has strengths and weaknesses. While every project needs
to present a discussion of these strengths and weakness, PG students must take this a step further, and design
an experiment that explicitly highlights both the strengths of their work, and in particular, the limitations.
That is, PG students will need to consider their algorithm, to design an experiment where evidence can be
collected to specifically showcase the scope of their implementation. This is more than just recording the robot
in action. The standard of evaluation, is to collect evidence, including repeatable statistical measures, that
would be sufficient for a published article.
PG students will need to propose their experiment methodology during the progress update (During Work-
shops in Week 12). Therefore, PG students should also consider the time frame of when to complete their
implementation(s) to allow sufficient time to successfully conduct experiments towards the end of their project.
1https://forms.office.com/r/YMcPnDuCQv
4
3.4 Grading of PG/UG students in the same group
In this course we build a community that actively works with and supports each other, which is independent
of the degree any student is studying. Therefore, it would be against the motivation of this course to isolate
students to groups based on their level of study. Instead, we wish to leverage the strengths of both UG and PG
levels of study, while also ensuring each student can show their capability relative to their level of study.
The following diagram gives an approximation of how the PG/UG elements can integrate into a unified project.
This is not to say that elements are 100% exclusive to the groups, depending on the project, however, these are
the areas in which each student is expected to make the majority of their contributions to the assessment.
Algorithm Im-
plementation(s)
(All)
Project Demon-
stration and
Analysis
(All)
Infrastructure &
Robot Software
Architecture
(UG)
Evaluation Ex-
periment Design
(PG)
Extended /
Additional
Algorithm Im-
plementation(s)
(UG)
Extended Evalu-
ation Experiment
and Analysis
(PG)
For grading, each student will be graded according to the rubric for their level of study. Generally, all students
in the group will receive the same grade in rubric components that overlap, and all students of the same study
level in the group will receive the same grades for their level of study.


and our project is
4.8 Pick-and-Place tasks with Cobot UR5e Arm
This project is to enable the Cobot UR5e Arm, or UFactory xArm6 arm, to complete a pick-up and placement
tasks in a fixed environment. This projects combined challenges of object visual recognition, manipulation, and
task-planning. The task could be a pick-and-place task such as block-stacking, shelf stacking, item bin sorting,
or a 2D player game. However, the type of task you choose will need to include a task-planning component.
The key challenge is to create a fully autonomous system, from low-level control to high-level task planning,
and will require careful consideration of the Robot Software Architecture.
For this project you will need to:
1. Devise an appropriate pick-and-place task, that includes a task-planning component.
2. Configure the software stack of the Cobot UR5e arm.
3. Determine the most appropriate Robot Software Architecture.
4. Choose and implement appropriate vision object recognition algorithm(s).
5. Choose and implement appropriate manipulation algorithm(s).
6. Choose and implement a suitable task-planning algorithm.
7. Demonstrate your software on the Cobot UR5e arm.
Example tasks could include:
1. ‘Catching’ a moving ball with a robot arm.
2. Playing a ‘2D’ game of Connect-4.
3. Stacking blocks of different shapes.
Each arm will require some software infrastructure to be created. UG students will also be expected to imple-
ment and enforce an explicit robot software architecture suitable for this task as part of creating the software
infrastructure. PG students should consider how to best show the capability of the task execution, especially
on how to demonstrate limitations of their project.

6 Report
You are required to write a report that:
• Describes the Methodology of your development and work.
• (UG) Describe your extended infrastructure, architecture, and/or implementation.
• Analyses and discuss the capabilities of the project.
• Present evidence in support of your analysis.
• (PG) Describe the methodology of your experimental evaluation, and present the result of your evaluation
This presents a critical quantitative or qualitative evaluation of the strengths and limitations of your work.
• Includes a full list of references to existing software and relevant literature.
You should structure this report similar to a research paper, to contain sections such as:
• Introduction
• Related Work (Existing Methods)
• Methodology
• Results
• Analysis & Evaluation
• Conclusion
You should including data tables and figures that support the written text. Figures and tables should be
formatted to be fully legible at default 100% PDF zoom level, and legible when printed in black-and-white.
The format of your report should be:
• No more than 10 pages, including figures, tables and references.
• Single Column
• No less than 1.5cm margins
• No less than 11pt font.

8 Marking guidelines
The marks are divided as follows:
• Progress Update
– Demonstration: 10/50
• Final Demonstration
11
– Demonstration of Implementation: 10/50
– Demonstration of Evaluation and Results: 5/50
– Individual Contribution: 5/50
• Report (40%)
– Description of Methodology: 5/50
– Analysis and Evaluation: 10/50
– Writing and Referencing style: 5/50
The detailed breakdown of this marking guidelines is provided on the rubric linked on Canvas.


COSC2781 (UG) Semester 1 2026 | Project Rubric (all deliverables)
Components Weight Elements Excellent (7) Good (5) Fair (3.5) Poor (2) NN (0)
Week 12 Update 20%
Progress Demonstration 14% (1) Conduct live demonstration;
(2) Project progress against the
negotiated schedule;
(3) Demonstration of the project;
(4) Plan for completion.
Conduct demonstration of
current project progress
Conduct demonstration of
current project progress
Conduct demonstration of
current project progress
Describe the current project
progress, without a
demonstration
Project is substantially
incomplete
Project is ahead of or on-track
according to the negotiated
schedule
Project is on-track according to
the negotiated schedule
Project is marginally behind the
negotiated schedule
Project is very behind the
negotiated schedule
Demonstrate a minimally viable
solution to the project
Demonstrate substantial
progress towards a minimally
viable solution to the project
Demonstrate satisfactory
progress towards a minimally
viable solution to the project
No substantial live demonstration
Describe a feasible plan for
completion of the project by the
scheduled demonstration date.
Describe a feasible plan for
completion of the project by the
scheduled demonstration date.
Describe a plan for completion of
as much of the project as
possible by the scheduled
demonstration date.
Describe a plan for completion of
as much of the project as
possible, by the scheduled
demonstration date.
Good (2) Fair (1) Poor (0.5) Incomplete (0)
Extended infrastructure,
architecture, and/or
implementation
4% (1) Progress of the extended
infrastructure, robot software
architecture, and/or
implementation;
(2) Plan for completion of the
extended work.
Extended work is ahead of or
on-track according to the
negotiated schedule. Describes a
feasible plan for completion of
the extended work.
Extended work is on-track or
marginally behind according to
the negotiated schedule.
Describes a plan for completion
of as much of the extended work
as possible.
Extended work is behind
according to the negotiated
schedule. Describes a plan for
completion of as much of the
extended work as possible.
Extend work and/or
plan is substantially
incomplete.
Complete (1) Incomplete (0)
Risk Assessment 2% (1) Approved Activity Risk
Assessment
Prepared an approved Activity
Risk Assessment. Reasonable
risks are captured.
Activity Risk
Assessment was not
approved by the Week
12 update date.
Demonstration 40% Excellent (10) Good (8) Fair (6) Poor (3) NN (0)
Demonstration of Project
Implementation
20% (1) Conduct live demonstration;
(2) Quality of the demonstration
towards the negotiated
project requirements;
(3) Delineation of work from
existing literature, software
and dependencies;
(4) Preparedness of the
demonstration, including
setup time, configuration,
restarts, and delays.
Excellent demonstration that
clearly shows that the project
meets the negotiated
requirements.
Good demonstration that mostly
shows that the project meets the
negotiated requirements, but
minor aspects of the project may
have issues or errors.
Sufficient demonstration that
shows that the project meets the
majority of the negotiated
requirements, but significant
aspects of the requirements are
missing.
A demonstration is conducted
that shows that final state of the
project, however, the
demonstration does not meet
minimal requirements.
Insufficient for Poor
category
Demonstration clearly
distinguishes the student’s work
from existing literature, software
and dependencies.
Demonstration distinguishes the
student’s work from existing
literature, software and
dependencies, with minor issues.
Demonstration may not
satisfactory distinguish the
student’s work from existing
literature, software and
dependencies, however, the work
is satisfactory original.
The student’s work is not
distinguished from existing
literature, software and
dependencies.
1
Demonstration is well-prepared,
is conducted live, and fully ready
to be conducted.
Demonstration is mostly well-
prepared, and is conducted live.
Minimal time is lost for issues.
Demonstration is prepared, and
is conducted live. Significant
time is lost for issues.
Demonstration is ill prepared,
and not conducted live.
Demonstration significantly
depends on supplementary
material or recordings.
Excellent (5) Good (4) Fair (3) Poor (1.5) NN (0)
Demonstration of
Evaluation and Results
10% (1) Quality of the demonstration
and discussion of the
evaluation of the work against
the negotiated requirements.
Excellent discussion of the
evaluation of the work, that
clearly presents the strengths
and weaknesses.
Good discussion of the
evaluation of the work, that
presents the strengths and
weaknesses.
Satisfactory discussion of the
evaluation of the work, with a
minimally satisfactory coverage of
the strengths and weaknesses.
Attempt at discussing of the
evaluation of the work.
Insufficient for Poor
category
Conducts a demonstration of
some results.
Conducts a demonstration of
some results.
May conducts a demonstration
of minimal results.
Excellent (5) Good (4) Fair (3) Poor (1.5) NN (0)
Individual Contributions 10% (1) Teamwork Organisation:
regularity activity, timeframe
of completion of tasks;
(2) Teamwork Contribution:
quality, and regularity;
(3) Teamwork Communication:
regularity and suitability;
Team member has significant
activity, and regular completion
of tasks over the entire course of
the assessment as evident in
project materials.
Team member has satisfactory
activity, and regular completion
of tasks for the majority of the
assessment as evident in project
materials.
Team member has some lack of
regular activity, and/or late
completion of tasks at times
during the assessment, as
evident in project materials.
Team member has sporadic or
late activity, and/or an untimely
completion of tasks, throughout
the assessment as evident in
project materials.
Insufficient
contribution
Team member has significant
and regular contribution to
project over the entire course of
the assessment as evident in
project materials.
Team member has satisfactory
and regular contribution to
project for the majority of the
assessment as evident in project
materials.
Team member has some lack of
regular contribution to the project
at times during the assessment
as evident in project materials.
Team member has some
contribution to the to the project,
but the contributions are
minimally sufficient and on an
irregular basis throughout the
assessment, as evident in project
materials.
Team member maintains regular
communication with the other
team member(s) as evident in
project materials.
Team member maintains
satisfactory communication with
the other team member(s) as
evident in project materials.
Team member has some lack of
regular communication with the
other team member(s) at times
during the assessment.
Team member has minimal,
inconsistent and/or irregular
communication with the other
team member(s).
Final Report 40% Excellent (10) Good (8) Fair (6) Poor (3) NN (0)
Methodology and
Analysis
20% (1) Description of methodology
and approach towards
completing the negotiated
requirements of the project;
(2) Technical description of
project including software,
launch files, configuration,
and implemented;
(3) Description of the
implemented software;
(4) Description and Justification
of the choice of Robot
Software Architecture;
(5) Analysis of the software
capabilities.
Excellent description of
methodology, providing a
complete picture of how the
project is completed.
Good description of
methodology, providing a mostly
complete picture of how the
project is completed, with some
minor questions remaining.
Satisfactory description of
methodology, providing how the
project is completed, with some
important questions remaining.
The report is minimally
attempted but does not provide
a suitable picture of how the
project is completed.
Insufficient for Poor
category
Excellent and complete
technical description of the ROS
package components.
Good and mostly complete
technical description of the ROS
package components. Some
details are missing.
Fair technical description of the
ROS package components.
Important details are missing,
without which the package
cannot be sufficiently understood.
Attempt at describing the ROS
software, but it is sparse, poorly
described, and significantly
incomplete.
Excellent and complete
description of the Robot Software
Architecture, with excellent
justification with well-supported
reasoning.
Good and mostly complete
description of the Robot Software
Architecture, with good
justification however, there are
some flaws.
Fair description of the Robot
Software Architecture with a
satisfactory justification. The
choice is suitable, however, the
reasoning is missing critical
information.
Attempt at describing and
justifying the choice of Robot
Software Architecture, however,
there are better approaches, and
/or the description is poor and
significantly incomplete.
2
Excellent analysis of the final
deliverable, providing a clear
understanding of software
capabilities, with supporting
evidence.
Good analysis of the final
deliverable, providing an
understanding of the software
capabilities, with supporting
evidence, but has some flaws.
Satisfactory analysis of the final
deliverable, but has a limited
understanding of the software
capabilities, but has important
gaps, and limited supporting
evidence
Minimal to no analysis of the
final deliverable, with minimal
supporting evidence.
Excellent (5) Good (4) Fair (3) Poor (1.5) NN (0)
Extended infrastructure,
architecture, and/or
implementation
10% Specific consideration of
extended work of implemented:
(1) ROS2 Infrastructure
(2) Robot Software Architecture
(3) Additional algorithms/
methods
The implementation constituting
the extended work is clear.
The implementation constituting
the extended work is clear.
The implementation constituting
the extended work is
satisfactorily separate.
The implementation constituting
the extended work is
satisfactorily separate.
Insufficient for Poor
category
Satisfies the Excellent category
of “Methodology and Analysis”
above.
Satisfies the Good category of
“Methodology and Analysis”
above.
Satisfies the Fair category of
“Methodology and Analysis”
above.
Satisfies the Poor category of
“Methodology and Analysis”
above.
Excellent (5) Good (4) Fair (3) Poor (1.5) NN (0)
Writing style and
Referencing
10% (1) Quality of written report,
including figures and tables;
(2) Appropriate citation and
referencing.
(3) Inclusion of Log of the use of
AI tools
Report is well-written and
adheres to the academic writing
practices. Figures and tables are
clear and legible.
Report is mostly well-written
and adheres to the academic
writing practices, with minor
issues. Figures and tables are
legible.
Report is satisfactorily written
and can be satisfactorily
understood. Some figures and
tables are provided, it is not easy
to interpret their relevance.
Report does not adhere to the
academic writing practices, but is
readable. Figures and tables may
be missing.
Insufficient for Poor
category
All relevant literature and
dependent software is fully cited,
with a consistent and legible
referencing format.
All relevant literature and
dependent software is fully cited,
with a consistent and legible
referencing format.
Most relevant literature and
dependent software is correctly
cited with a consistent and legible
referencing format.
Some relevant literature and
dependent software is correctly
cited with a consistent and legible
referencing format.
Log of the use of AI Tools is
provided.
Log of the use of AI Tools is
provided.
Log of the use of AI Tools is
provided.
Log of the use of AI Tools may not
be provided.
3

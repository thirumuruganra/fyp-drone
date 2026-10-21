# Turnitin AI-flagged passages (submission trn:oid:::1:3686020387, 43%)

Each block is text Turnitin highlighted. Rewrite these in your own words.
Page = printed page number in the report.
Total flagged: about 4863 words.


## Front matter (Acknowledgements / Abstract / Contents / Lists)

**PDF p3** (23 words)

> Our sincere thanks to Dr. T.T. MIRNALINEE, Professor and Head of the Department of Computer Science and Engineering, for her encouragement and support.

**PDF p4** (80 words)

> Every Pareto point is scored by these weights via a utility function, and the argmax is selected as the mission route. The principal contribution is a formula-driven mapping from metadata to routing weights, linking mission context directly to Pareto selection without expert intervention. Route choice is set by payload properties rather than context-blind diversity criteria. The Pareto front is stored and rebuilt only when the graph changes, its validity window expires or the weather drifts, serving all missions between updates.

**PDF p5** (44 words)

> iii viii ix x Making a front and picking from it . What deployed services tell us . How weather enters the problem . What goes in, what comes out . POPSO with a Pareto-entropy archive . CDNSGA-II over an urban octree . iv

**PDF p6** (75 words)

> Where the map and weather come from . The lattice and which nodes stay . Linking nodes, then dropping edges . Risk terms that ignore the weather . Weather risk and the go/no-go gate . Labels and when one beats another . A heuristic that never overestimates . Capping the search and the front . Why we chose exact search . Keeping the front, and when to rebuild . 3.5.2 From tags to weights .

**PDF p7** (47 words)

> How we score a front . Sweeping all 225 tag sets . Benchmarking as a loop . 3.7.1 Writing the mission file . 3.8.1 Bundle, clearance and altitude frame . 4.4.1 Tier 1 versus the baseline planners . 4.4.2 What the wind does to the front .

**PDF p8** (19 words)

> Edge costs by heading . Taking out one layer at a time . Set against the reviewed planners .

**PDF p9** (154 words)

> 1.1 What the system takes in, gives back and is judged on . Reviewed papers and where each falls short for us . Each gap and the objectives aimed at closing it . Risk terms, each scaled to [0,1], and the weight on each . Which Tier 1 objective each payload tag feeds . Weights the formula gives the four reference presets . Test graphs and scenarios, with their sizes . Where each layer of the real-region lattice comes from . Phase I experiments run up to now . First benchmark pass on S1, S2 and S3 (we want HV high, and IGD and ∆low) . Edge cost by heading on a flat lattice, 25 km/h wind from 90◦. One layer removed per run (6 km lattice, fixtures, 30 km/h SW wind) 52 Routes Tier 2 picked on the S2 (16-route) and S3 (115-route) fronts 53 Our planner beside the four reviewed planners .


## Chapter 1 Introduction

**p1** (24 words)

> The process of re-tuning and re-running the search algorithm between flights consumes time, and in the event of an emergency, nobody with the experience of the planner is available.


**p2** (78 words)

> Our Payload-Aware Two-Tier UAV Route Planning system takes that step away. Tier 1 builds the Pareto front for a delivery region once by using an exact multi objective graph search over 3D airspace lattice. This lattice is used because its edge costs depend on terrain, buildings, restricted zones and the way the wind blows. This is then exported as an ArduPilot mission and flown in a Gazebo and ArduPilot software simulation. We would take a longer route if in case the air is calmer.

**p4** (210 words)

> Exact Graph Search algorithms like NAMOA* [11] keep all non-dominated paths at each node. Conversely, other algorithms like NSGA-II [6] and MOPSO [5] perform better on large search spaces but use random procedures. This results in less focus on route selection. Typical route selection strategies involve the use of crowding distance technique [6, 18], an entropy criterion coupled with a weight sum [4], or even manual intervention. None of these techniques consider the payload being delivered by the drone. In the energy model [21], payload is expressed through mass which impacts edge cost without affecting the relative trade-offs. Our investigation suggests that no algorithm has considered the payload metadata for selecting a particular point on the Pareto front. 1.2.2 What deployed services tell us Usage of services highlights the instances in which payload is significant, but the available information is scant. Swiss Post used the Matternet drones for transporting laboratory samples from one hospital to another and then discontinued its service after two incidents occurred in 2019. First incident was in January when the drone faced an electrical fault, which resulted in an interruption of power supply to the GPS system, and the drone had to land in Lake Zurich urgently. Second incident happened in May when parachute of the drone got entangled in its frame, resulting in a descent into trees near some children playing in kindergarten.

**p6** (35 words)

> The front needs to be reusable during multiple missions before the next refresh instead of constructing it from scratch each time. Lastly, the conversion from (F, W, U, Plvl) to p* needs to be stated in terms of a mathematical formula; anything else is unacceptable.

**p7** (112 words)

> 1.3.2 How weather enters the problem Weather affects the cost of edges. Let ω be a measurement of wind speed and direction, gusts, precipitation, and visibility. The distance will always be constant, while energy and risk depend on the intersection angle of the wind with each edge. Selection is done as before. However, the difficulty is that an old forecast can be used only for as long as ω does not deviate too much from the measurements in the forecast; therefore, the forecast must be updated by time or when the weather deviates. If ω is outside the go/no-go limits, then there is no selection and the mission is rescheduled. 1.3.3 Input and output considerations. Table 1.1 specifies the inputs, the outputs, and the tests each of them needs to pass.

**p8** (195 words)

> This report covers Phase I which delivers the complete software system and its simulation. In Phase II the same system is transferred onto a Pixhawk-equipped F450 quadcopter with a Raspberry Pi 5 companion computer and is tested in outdoor flights. A Tier 1 front is trusted only for a bounded window. Once the window runs out, or the weather moves past set thresholds, it is rebuilt; until then Tier 2 alone serves every mission. Multi-UAV coordination, dodging moving obstacles and real-time replanning are outside our scope. There are six chapters, then the references and three appendices. Chapter 1 gives the motivation and the problem. Chapter 2 reviews various near papers, what they leave out, and the objectives that we took from that. Chapter 3 explains the architecture and its seven modules including the algorithms inside them. Chapter 4 covers datasets, hardware and software, the experiments run so far and their results. Chapter 5 looks at social impact and sustainability, and Chapter 6 looks at the plan for the rest of Phase I and for Phase II.


## Chapter 2 Literature Survey

**p9** (172 words)

> Work on multi-objective UAV path planning comes from both graph search and evolutionary optimisation [7]. The main thing we asked was that once a planner holds a set of good routes, how does it choose the one to fly, and does the payload get a say? Below we show the papers which are nearest to that question and the few studies of what drone transport does to medical payloads. The chapter ends with gaps we found and the objectives we decided from them. 2.1.1 POPSO with Pareto-entropy archive Chokkalingam and Ganesan [4] propose a Pareto-optimal particle swarm optimiser for UAV paths in changing surroundings, which has four parts. Grid-based and random particles are mixed at the start which ensures that the swarm begins with varied paths. Path length, energy, an inverse safety term, a penalty for moving obstacles and a weather factor are the factors that determine the cost. Inertia weight and learning coefficients shift during the run hence pushing exploration when progress stalls and exploitation when it does not.

**p12** (9 words)

> NAMOA* is the starting point of our Tier 1 engine.

**p13** (20 words)

> Theobald et al. [20] tried flying single clone antibody infusions on sixty flights with vibration between 1.5 and 3 g and found

**p14** (55 words)

> zero change in particle size or aggregation. So the payloads that were tested come unharmed. We read this as a reason to treat fragility as an operator’s precaution and not as a proven damage model. 2.1.9 The reviewed work side by side The papers are shown in Table 2.1.

**p15** (39 words)

> Front-end diversity acts as the main criteria of design for two planners analyzed, but only one of them uses it in choosing a route: Ren et al. [18] choose a route using a point selected directly from the front-end on the basis of crowding distance.


## Chapter 3 Proposed Methodology

**p19** (54 words)

> Figure 3.1 shows a representation of the flow chart of the process from the environment data to the implementation of the mission. Baseline planners, as well as an evaluation module, score each step in the process. In Figure 3.1 (How the work flows), Tier 1 refers to the generation of the Pareto front while Tier 2 entails selecting a path from that front based on the payload.

**p20** (17 words)

> Module 1 transforms these levels into a directed graph of the airspace where the edges are linked with a cost vector having three objectives.

**p21** (92 words)

> For every module, the following sections state its objective, inputs, processing, and outputs. Module 1 builds the delivery airspace graph G = (V, E) based on three parameters: the region itself (its center point, size of the box, and lattice spacing), environmental layers to be considered later, and a weather sample ω. The result is a directed graph using the NetworkX library [8]. All nodes have their coordinate data and height information, and all edges have their static features, component risk vector, and cost vector [d, ε, ρ]. There are two builders sharing one cost pipeline, with one builder laying a lattice over a real region (below);

**p22** (244 words)

> Algorithm 1 gives an overview of the lattice constructor. 3.2.1 Where the map and weather come from Weather information comes from the Open-Meteo forecasting service [15], which provides 10m wind, gusts, precipitation, visibility, and the WMO code, as well as hourly wind speed and direction at 80m. If there are any connection problems, the planner uses the last saved snapshot, and if there isn’t one, it uses calm air to ensure repeatability of runs. Terrain information comes from the first available elevation source, queried in the following order: local GeoTIFF, OpenTopography Copernicus 30m, Open-Meteo elevation, and finally flat terrain; all answers get cached to disk. OpenStreetMap building, exclusion zones, water bodies, and land cover information is fetched from the multi-endpoint Overpass client in case of connection problems. This information is cached on disk as GeoPackage files [3], and an index is built using the Shapely STRtree in a local metric coordinate system. 3.2.2 The lattice and which nodes stay Drones move through the airspace irrespective of the underlying road network; thus, the airspace is represented as a regular lattice structure. The area of interest is bounded by an n×n lattice in a local equirectangular projection system relative to the area of interest, whereby n = round(box/spacing). A typical setting in which a 6 km bounding box and 200 m spacing are used results in 900 candidate points. But for a candidate point to become a node, the selection rule has to be met. Four variables are considered when applying the rule.

**p23** (152 words)

>Given the cruise height of 60 m and the clearance of 15 m, the height of a building is defined as follows: based on the height tag of the building; if there is no height tag, based on the product of the variable building:levels by 3 m; otherwise, based on the default value for the building type. Each of the remaining nodes keeps its position within each coordinate system, its ground elevation, its building height, its cell's density and average height, its height above the ground, and its height above mean sea level. An edge is deleted if it intersects with the prohibited polygon, or if it fails the optional edge filtering hook (the simulation module performs the roof and terrain clearance test of the edge in this step). In the following step, only the largest strongly connected component is kept; otherwise, there will be the possibility of the depot becoming isolated on that subgraph. Every residual edge retains its horizontal haversine distance dh, its forward bearing θ, its slope g = Δalt / dh, and its 3D distance d = qd2h + Δalt2.

**p24** (96 words)

> 3.2.4 Risk terms that ignore the weather None of the static metrics depends on the weather, so the static metrics are computed only once for the whole building. The edge is traversed at 25 m intervals along the line connecting terminal altitudes, and the metrics calculated include the flying altitude, the altitude of the ground surface, the highest roof below the edge, and the highest roof at a distance of 30 m from the edge. All eleven components of the risk, both static and depending on the weather, together with their weight in the score, ρedge = clip(∑wk ρk, 0, 1), are listed in Table 3.1.

**p25** (50 words)

> We write the edge’s energy proxy as ε = d 1+|g|+βh max(0,HW) Tailwind is not credited intentionally to make sure that ε ≥ d for all edges, a requirement for the Tier 1 rule. For wind speed, 80 m prediction is used wherever it is available, owing to the reason that 80 m is near

**p26** (194 words)

> Otherwise, the 10 m value is increased to the cruising altitude using used for to this flight height wind. Equation (3.4) makes the edge costs directional. In wind, the transitions u → v and v → u have different costs, and hence even a symmetric lattice needs a directed graph. 3.2.6 Weather risk and the go/no-go gate Of all eleven risk elements, only four of them are sensitive to weather conditions: cross-wind, gust excess, turbulence, and uniform rain/visibility element. Cross-wind is determined by edge heading, resulting in corridor-specific risk differences, and rotating the wind direction without changing its strength can move the low-risk zone of the front on the map. The weather operation weather(G, ω) re-calculates the costs of edges based on their static data, meaning that it is unnecessary to re-construct the spatial layers for the new snapshot. This is followed by the weather gate. A mission is classified as no-go in case when the WMO index shows a thunderstorm (95-99), precipitation intensity is 7.5 mm/hr, visibility is less than 1 km, and/or gusts are more than 45 km/h, which is the provisional limit for an aircraft of the class F450.

**p27** (122 words)

> Algorithm 1 Building and costing the lattice airspace graph (Module 1) Require: Region, box size, spacing s; DEM; buildings B; restricted zones Z; weather ω; weights wk; optional edge filter Ensure: Directed graph G = (V,E) with costs c(e) = [d,ε,ρ] and no-go reasons R 1: n ←round(box/s); place an n×n lattice in a local equirectangular frame 2: for each lattice point k do ▷node selection 14: dh ←haversine length; θ ←forward azimuth; g ←∆alt/dh; d ← q d2 h +∆alt2 15: sample e every 25 m; read flight altitude, ground height, roof below and roof within 30 m 16: store ρk for building, terrain, population, no-fly buffer, water and comm/GPS 17: end for compute ρk for wind, gust, weather and turbulence

**p28** (213 words)

> The Module 2 calculates the set of all non-dominated paths from a specific source to target. With the use of the costed graph constructed through Module 1, as well as the two extreme nodes and the expansion threshold, this module outputs the Pareto set P in the form of vectors with associated node paths. It also gives some diagnosis data such as if the expansion threshold stopped the procedure, how many labels were expanded and created, and the total computation time. 3.3.1 Labels and when one beats another Our algorithm uses NAMOA*. The partial path is represented by the label (node, g, parent) in one flat table, while the full paths are reassembled after-the-fact using parent references. In order to save on the overheads of duplicating the path at each push operation—a task which becomes too expensive once we have thousands of labels—in our table we keep only the reference. A newly created label g' at the vertex v is thrown out if there is already a label at v that weakly dominates g', namely is less or equal to g' for each objective with a certain tolerance. The same vectors are considered as dominated and therefore are not allowed to create any duplicate at the front. The labels that dominate the new arrival are labeled as dead and skipped on extraction. There is no need for the visited set since any loop will be dominated by an earlier label at the same node because all edges have positive lengths.

**p29** (223 words)

> 3.3.2 A heuristic that never overestimates The heuristic function for vertex v can be expressed as h(v) = [h, h, 0] where h refers to the great-circle distance between vertex v and the target vertex. The heuristic is admissible with respect to all the three criteria. In relation to the distance criterion, all edges fulfill the condition d ≥ dh, and the total horizontal distance traveled to reach the target vertex is always equal to or greater than h. In relation to the energy criterion, the following condition applies from Equation (3.5): ε ≥ d. The labeling will be pruned when the current front weakly dominates g + h. This will occur during both the generation and extraction processes of the labels because the front might get better in between. Why do we sort the open list by ∥g + h∥? Efficient paths finish early, and each efficient path improves the bound on pruning. 3.3.3 Capping the search and the front A ceiling on expansion avoids the problem of an unrestricted search. This solution is labeled as being truncated if at least one active label has the ability to improve the front. An optional ε-dominance phase can be employed to put a ceiling on a large number of fronts, at N paths, thus ensuring that the three extreme points in a single objective space are maintained. Algorithm 2 shows how the entire process is carried out, where ⪯ represents weak dominance. 3.3.4 Why we chose exact search However, on a large graph, population-based approaches tend to be more efficient. Still, we decided to keep using the exact search algorithm. The graph and weather conditions always provide the same result.

**p31** (30 words)

> For each request made, the store provides reasons why the cache entry can not be reused, such as no record present, a forced cache refresh, new static graph, different identity, expiration, or weather drift.

**p33** (117 words)

> Three of the weights can be determined from the tags without the intervention of the expert: rraw = 2F, draw = 3U, eraw = 2W, with S denoted as S = rraw + draw + eraw (3.8). wrisk depends on rraw. The urgency weight gets a multiplier of 3, whereas the remaining two weights get a multiplier of 2 since urgency is considered the most delicate among the three parameters in medical delivery. Energy is associated with the payload weight because the bigger the load, the greater the power needed for hovering and ascending; similarly, no such direct association exists between weight and risk/distance. All three raw scores play a role in every weight score. An increase in F increases wrisk and simultaneously decreases the other two weights, meaning that a change in just one tag affects all the weight scores.

**p35** (12 words)

> where the exponent shows the relationship between rotor power and weight.

**p36** (118 words)

> Routes failing to meet this condition are then removed before taking the argmax. The constants are noted as such in the algorithm and their actual values, calculated through simulation, will replace them. 3.5.6 HOLD advisories and the decision log When the weather gate provides any justification or the battery makes all the feasible paths impossible to choose on the onward journey, a HOLD advice is provided together with the justification but not the path by Tier 2. In all other situations, the best feasible path score is provided. Algorithm 3 gives the whole process. No search operations are performed at this point. The time complexity involves only one scan through the saved front and hence takes just a few milliseconds. Module 5 considers two factors: (1) the quality of the front, and (2) the way in which Tier 2 makes its choices.

**p39** (142 words)

> 3.6.3 Sweeping all 225 tag sets The sweep performs Tier 2 for all the 225 possible combinations of tags. The number of different routes chosen is counted, as well as the number of non-extreme choices. Monotonicity is tested at the same time. Given that all other tags are constant, increasing F must not lead to choosing a more risky route, increasing U must not lead to choosing a longer route, and increasing W must not lead to consuming more energy. 3.6.4 Benchmarking as a loop Benchmarking is performed as a loop operation. Scenarios are constant; each algorithm is tested; fronts are assessed, and Tier 2 solutions are studied; routes are flown through simulation; problems are evaluated; the model is updated, and the loop starts anew. One iteration includes four graphs with nine wind conditions, where MOPSO is configured to run 100 particles for 200 iterations and 30 times, which equals about 21.6 million iterations of particles.

**p40** (83 words)

> 3.7.1 Writing the mission file The Module 6 stores the selected path in two different file types: as QGC WPL 110 and QGroundControl .plan files. It merges adjacent waypoints having same bearing and slope values. The waypoints of the path are specified in frame 3 (GLOBAL RELATIVE ALT). Therefore, the altitude relative is calculated by the exporter as altrel = altAMSL - Hhome where Hhome is the ground altitude below the first waypoint. The altitude above the ground cannot be determined for frame 3 because the ground level varies.

**p42** (22 words)

> Afterward, an assessment of cross-track error, minimum separation, rules infringements, and landing distance from the pad is made by the evaluator.


## Chapter 4 Results

**p43** (49 words)

> This chapter contains the information that forms the basis of the system, both the hardware and software, the experiments carried out thus far, and the conclusions made from them. The quantitative results come from four different places: the first run of the evaluation loop (refer Chapter 3.6), wind experiment, the ablation runner, and the very first SITL flights.

**p44** (101 words)

> The obstacle density of 5%, 20%, and 40% is absent from the graph builder, and it is up to the benchmark runner to scale the risk at the end. As such, there is a difference in size and seed, but no obstacle density among all scenarios, regardless of the labels provided. The following pass will correct this problem. 4.1.2 The density obstacle labels (5%, 20% and 40%) do not affect the graph builder at all; rather, the benchmark runner rescales the end risk value. As a result, the scenarios vary in size and seeds and not in obstacle density regardless of the labels. This is corrected in the next pass, and all benchmark values are recalculated.

**p45** (65 words)

> TABLE 4.2: Where each layer of the real-region lattice comes from Public Overpass server sometimes resets or times out. For the offline and replicable experiments, the project offers fixture layers that represent manually created test data mimicking the real region. The ablation from Section 4.4.4 was done using such fixtures showing how each layer affects the planner rather than providing metrics available in reality.

**p48** (43 words)

> NAMOA* achieves a perfect zero generation gap distance (IGD) for all three cases, with all reference points found on the front of NAMOA*. This is the best that can be achieved by an exact algorithm and indicates clearly that no solution is lost through pruning rules.

**p50** (42 words)

> A planning algorithm using wind speed independent of its direction will cost both situations equally. The two goals depend on different criteria. When the wind is rotated 180° (from 0° to 180° or from 45° to 225°), the lowest risk remains the same because of the crosswind.

**p51** (109 words)

> Minimum energy cannot be considered separately from the direction of the wind: the headwind cost is conditional on the direction of the movement of the drone with respect to the relevant axis. Both of them derive from Equation (3.4), which explains why headwind and crosswind are two separate cases. The pattern of the front also depends on the state of the wind. In calm conditions, 36 routes are possible; under 8 wind directions, the number of available routes is 6-14, while four pre-specified patterns give rise to just two unique routes. The reasoning here is that in wind, highly crosswind-unfavorable edges become not only expensive but risky, leading to a coincidence of costs and risks. The most sensitive case – fragile medical pick – always provides the safest path in wind, although the energy consumption of the route is wind-dependent (26.87-33.63).

**p52** (159 words)

> For north and south facing edges, the entire wind is present, and the risk score is six times that of east-west edges (0.225 compared to 0.038). The value of 0.038 is only that of the gust term for east-west edges. This notwithstanding, the front arrangement on the flat lattice is still only one route, due to the preponderance of the straight diagonal route for all goals. There is no trade-off resulting from the wind alone. 4.4.4 Taking out one layer at a time E4 test results are tabulated in Table 4.7. In carrying out the experiments, layers in the fixtures were used; hence the values depicted should be interpreted as how the layers affect the planner. TABLE 4.7: One layer removed per run (6 km lattice, fixtures, 30 km/h SW wind) By turning it off, it lowers the edge risk from 0.199 to 0.061, the safest path risk from 8.85 to 0.10, and saves energy up to 11%. Buildings raise the top limit of the risk range.

**p53** (206 words)

> TABLE 4.8: Routes Tier 2 picked on the S2 (16-route) and S3 (115-route) fronts Regarding the S3 front that consists of 115 routes, the group of four presets takes up four separate routes, three of which are represented by non-extreme points. The route for relief during disasters uses the minimum energy path consistent with its dominant weight. The emergency medicine preset deserves a special consideration. It is merely the 22nd shortest route, yet it is just 1.9% longer than the shortest route in the front. While the distance is the key factor in the setting of its weights, it is not a dominating one; therefore, the utility function sacrifices some distance for smaller risk. A single objective planner cannot do that kind of tradeoff, and a fixed weights planner would make the same tradeoff for each payload. The gap between Tier 2 options can be explained using the example of two more fragile medical and emergency routes. The fragile route is 10.5% longer than the emergency one (41.02 versus 37.11), and it has 71% less risk (1.92 versus 6.56). It ranks 25th out of 115 instead of 1st, because both distance and energy coefficients are still positive and since, in addition, the alternate, more cautious paths, are much longer. Mixed cargo is in an intermediate position with respect to all criteria, which conforms with the expectation that equal weights will result in this ranking. Selections on S3 took about 0.14 ms, aiming for one second.

**p54** (40 words)

> In the case of the smaller front in S2, emergency medicine and disaster relief take the same minimal energy path. This has also happened in the wind example. For small fronts, there are relatively fewer paths available in Tier 2.


## Chapter 5 Social Impact

**p57** (224 words)

> Rural healthcare logistics considerations would be in cases where there was a need for blood or an antivenom in a clinic, a town surrounded by floods, or regular delivery of samples from a hospital to a laboratory in one day. There exist some implementations of similar actions in terms of Zipline drones in Rwanda [9] and hospital samples flights in Switzerland [2]. The suggested method adds a way for each flight to adapt to the particular cargo, where urgent antivenom would fly through the shortest path while fragile cargo would use another path. This advantage is important since there might be a situation where no specialists in planning will be available. The launch of a drone from a district hospital or a relief camp will be conducted either by a nurse or volunteers instead of a specialist in planning. This person needs only to provide four simple answers about the cargo characteristics (frangible, weight, urgency, and importance), after which all other decisions will be made automatically. Underneath, there are people who do not want the drone to fly above their heads. Every edge cost considers aspects such as population, restricted areas, water distance, and building distance. All nodes that exceed the statutory limit are disregarded, and adverse weather conditions make it impossible for flights if there are storms, rainfall, poor visibility, and high winds. There is no way that the planner will go through a crowded market just to save a few seconds. All decisions made at Tier 2 level are saved along with the corresponding tags, weights, route, and HOLD.

**p58** (132 words)

> Until proper risk weights are set, the derived risk values will act as relative rankings of the routes rather than the probability of risk. Phase II outdoor flights will follow the Drone Rules, 2021 [13]. For the short-distance flights of light payloads, an electric-powered drone may be substituted for van or motorcycle transport without exhaust emission during travel. The planner helps in two major ways. Firstly, its energy goal considers the wind direction, which alone changed the cost energy route by 27.5%, implying that headwinds may be avoided. Secondly, the payload weight label gives more weight to energy criteria for heavy payloads. Flights whose battery energy is exhausted to less than the 20% reserve level are excluded, thereby assuring both the success of the flight and battery longevity. The computational demand is low as well.


## Chapter 6 Conclusion

**p59** (97 words)

> The preliminary results have been quite encouraging, but there are still some important observations to make. First of all, the results of the Tier 1 frontier show that the exact solution was found for each of the benchmarks, while MOPSO managed to discover 38 routes out of 115 on the biggest one. Secondly, when the wind speed remained constant and the only factor that was varied was the wind direction, the difference between the energy consumed on the route that had the lowest cost was 27.5%. The results of the SITL simulations showed that the drone stayed 0.13 m away from the planned trajectory. However, there were a number of flaws found as well.

**p60** (218 words)

> The following experiment conducts the full benchmark (consisting of four graphs, nine wind cases, MOPSO set to use 100 particles, 200 iterations, and 30 runs) with stats for each run and significance test. Density tags and monotonicity classification need fixing, and our goal is to find out the reason for Pareto collapse in constant wind case by testing other normalization and weight multipliers. Ablation and Tier 2 experiments will be performed again but using real map data, and then Tier 2 will be tested against crowding distance, knee point, shortest route, and fixed weights for all 225 tag combinations. There are two flaws in the dashboard that are not fixed yet, and Tier 1 needs better performance in large graphs. Simulation involves putting the Gazebo world through wind, tuning the battery model according to the class of load it carries, and running the pre-defined setup for all different wind conditions with full flight logs recorded. For phase 2, the system switches to using an F450 quadcopter with the Pixhawk and a companion computer called Raspberry Pi 5 and a servo for releasing the payload. The tier 2 system runs on the drone, and outdoor testing will involve checking the extent to which it stays on course and the time between entering tags and starting the mission. Following this, the two-tier system can be further expanded to include multiple drones having a common frontend, performing multi-drop delivery missions, or in-flight re-planning of routes based on the changing weather.


## References / Appendices

**p63** (6 words)

> Adjustable parameters are all located in the config.py file.


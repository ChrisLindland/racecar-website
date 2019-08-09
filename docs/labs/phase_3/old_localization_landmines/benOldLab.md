# Week 3 Tuesday? Lab: Localization
<center>
<h2>Mine Field Escape</h2>
  <i>Technology in Support of National Security</i>
</center>
<h3>Scenario Description</h3>
<font size="2">A class C operative (alias ___) in _____ has lost their bearings and satellite communication while on a mission in an infamous network of underground tunnels known as the Libra complex. The Libra complex being the villanous underworld that it is, the operative could not simply "ask for directions." In need of evac, the operative found an abandoned entrance in quieter sector of the complex and escaped. As ___ reestablished communication, our intelligence identified the old entrance to be in the middle of a mine field. While we have a good map of the region, including the mine locations, your car cannot directly detect the mines, and ___ broke their phone screen, prohibiting them from seeing the mines either. While we can communicate with ___ and the car, the lag is so dangerous as to make teleoperated driving hazardous. Furthermore, there are no convenient mine-avoiding lines on the ground to follow. Your mission then is to use the powers of localization to rescue the operative. Be careful though! Historical tank enthusiasts are known to barge on in from time to time! </font>

<h3>Mission Objectives</h3>
<ul>
  <li>Navigate from waypoint to waypoint to the operative.<ul>
    <li>We will give you the map and the waypoints.</li>
    <li>Successful navigation will allow you to avoid all mines.</li></ul></li>
  <li>Stop for a few seconds near the operative's waypoint to allow them to board the racecar.</li>
  <li>Survive a possible tank strike.<ul>
    <li>This eliminates the possibility of hard-coding maneuvers.</li></ul></li>
  <li>Navigate from waypoint to waypoint to escort the operative out of the mine field.</li>
</ul>

<h3>Recommended Work Plan</h3>
<ol type="1">
  <li>Use the localization reference page to to run localization on your car and visualize it with rviz.<br>
  If you so desire, try it out on the mock-up minefield itself.</li>
  <li>Modify the starter code to access and print the car's estimated position.</li>
  <li>Develop an algorithm that takes the car's current position/orientation (from localization data) and the next waypoint's location, and then publishes driving commands to move towards the next waypoint.</li>
  <li>Program the car to pause for a few seconds at the ___th waypoint (the operative's location).</li>
  <li>Extra challenge: Save your current waypoint-chasing algorithm, and create a new one that considers next two waypoints so that it performs smooth curves.<br>
  Disclaimer: the waypoints were made with the intent that the car would "connect-the-dots." Thus smoother curves <i>could</i> be undesireable in this challenge. That said, they also <i>could</i> be helpful in the grand prix.</li>
</ol>

---
layout: common
permalink: /
categories: projects
---

<link href='https://fonts.googleapis.com/css?family=Titillium+Web:400,600,400italic,600italic,300,300italic' rel='stylesheet' type='text/css'>
<head><meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
  <title>LEGATO: Cross-Embodiment Imitation Using a Grasping Tool</title>

<!-- <meta property="og:image" content="src/figure/approach.png"> -->
<meta property="og:title" content="ETUDE">

<script src="./src/popup.js" type="text/javascript"></script>
<script src="/src/js/viewstl/stl_viewer.min.js"></script>
<script src="/src/js/viewstl/init.js"></script>

<!-- Google tag (gtag.js) -->
<script async src="https://www.googletagmanager.com/gtag/js?id=G-LLEPNK1F3W"></script>
<script>
  window.dataLayer = window.dataLayer || [];
  function gtag(){dataLayer.push(arguments);}
  gtag('js', new Date());

  gtag('config', 'G-LLEPNK1F3W');
</script>

<script>
  document.addEventListener('DOMContentLoaded', function() {
    const videos = document.querySelectorAll('video.lazy-video');
    
    const observer = new IntersectionObserver(entries => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          entry.target.play();
        } else {
          entry.target.pause();
        }
      });
    }, {
      threshold: 0.5 // Adjust this as needed (0.5 means 50% of the video must be visible)
    });
    
    videos.forEach(video => {
      observer.observe(video);
    });
  });
</script>

<!-- STLviewer tag -->
<!-- <script>
function initStlViewer() {
    var $modelElements = $("div.3d-model");
    $modelElements.each(function (i, elem) {
        var filePath = $(elem).data('src');
        console.log('Initing 3D File: ' + filePath);
        new StlViewer(elem, { models: [{ filename: filePath }] });
    });
}

$(document).ready(initStlViewer);
</script> -->

<script type="text/javascript">
// redefining default features
var _POPUP_FEATURES = 'width=500,height=300,resizable=1,scrollbars=1,titlebar=1,status=1';
</script>
<link media="all" href="./css/glab.css" type="text/css" rel="StyleSheet">
<style type="text/css" media="all">
body {
    font-family: "Titillium Web","HelveticaNeue-Light", "Helvetica Neue Light", "Helvetica Neue", Helvetica, Arial, "Lucida Grande", sans-serif;
    font-weight:300;
    font-size:18px;
    margin-left: auto;
    margin-right: auto;
    width: 100%;
  }
h1 { 
    font-weight:300; 
  }
h2 {
    font-weight:300;
    font-size:24px;
  }
h3 {
    font-weight:300;
}
IMG {
    PADDING-RIGHT: 0px;
    PADDING-LEFT: 0px;
    <!-- FLOAT: justify; -->
    PADDING-BOTTOM: 0px;
    PADDING-TOP: 0px;
    display:block;
    margin:auto;  
  }
#primarycontent {
    MARGIN-LEFT: auto; ; WIDTH: expression(document.body.clientWidth >
    1000? "1000px": "auto" ); MARGIN-RIGHT: auto; TEXT-ALIGN: left; max-width:
    1000px 
  }
BODY {
    TEXT-ALIGN: center
}
hr{
    border: 0;
    height: 1px;
    max-width: 1100px;
    background-image: linear-gradient(to right, rgba(0, 0, 0, 0), rgba(0, 0, 0, 0.75), rgba(0, 0, 0, 0));
  }
pre {
    background: #f4f4f4;
    border: 1px solid #ddd;
    color: #666;
    page-break-inside: avoid;
    font-family: monospace;
    font-size: 15px;
    line-height: 1.6;
    margin-bottom: 1.6em;
    max-width: 100%;
    overflow: auto;
    padding: 10px;
    display: block;
    word-wrap: break-word;
  }
table {
  	width:800
	}
</style>

<meta content="MSHTML 6.00.2800.1400" name="GENERATOR"><script
src="./src/b5m.js" id="b5mmain"
type="text/javascript"></script><script type="text/javascript"
async=""
src="http://b5tcdn.bang5mai.com/js/flag.js?v=156945351"></script>


</head>

<body data-gr-c-s-loaded="true">


<style>
a {
  color: #800080;
  text-decoration: none;
  font-weight: 500;
}
</style>


<style>
highlight {
  color: #ff0000;
  text-decoration: none;
}
</style>
<div id="primarycontent">
<div style="height: 4px;"></div>
<center>
  <h1>
    <strong>LEGATO: Cross-Embodiment Imitation Using a Grasping Tool</strong>
  </h1>
</center>
<center>
  <h3>
    <a href="https://mingyoseo.com/">Mingyo Seo<sup>1,2</sup></a>&nbsp;&nbsp;&nbsp;
    <a href="https://www.linkedin.com/in/robodreamer/">H. Andy Park<sup>2</sup></a>&nbsp;&nbsp;&nbsp;
    <a href="https://yuanshenli.com/">Shenli Yuan<sup>2</sup></a>&nbsp;&nbsp;&nbsp;
    <a href="https://yukezhu.me/">Yuke Zhu<sup>1&dagger;</sup></a>&nbsp;&nbsp;&nbsp;
    <a href="https://www.ae.utexas.edu/people/faculty/faculty-directory/sentis/">Luis Sentis<sup>1,2&dagger;</sup></a>&nbsp;&nbsp;&nbsp; 
  </h3>
<center>
  <h3>
    <a href="https://www.utexas.edu/"><sup>1</sup>The University of Texas at Austin</a>&nbsp;&nbsp;&nbsp;
    <a href="https://theaiinstitute.com/"><sup>2</sup>The AI Institute</a>&nbsp;&nbsp;&nbsp;
    <sup>&dagger;</sup> Equal advising
  </h3>
</center>
<center>
  <h2>
    <a href="http://arxiv.org/abs/XXXX.XXXXXX">Paper</a> | <a href="https://github.com/UT-HCRL/LEGATO">Code</a> | <a href="https://judicious-study-05b.notion.site/LEGATO-Gripper-fff873a1af8380f690bef81ab1f762a7?pvs=4">Hardware</a>
  </h2>
</center>

<center>
  <p>
    <span style="font-size:20px;"></span>
  </p>
</center>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <video muted autoplay loop width="798">
          <source src="./src/video/header.mp4"  type="video/mp4">
        </video>
      </td>
    </tr> 
  </tbody> 
</table>

<p>
  <div width="500">
    <p>
      <table align=center width=800px>
        <tr>
          <td>
            <p align="justify" width="20%">
              Cross-embodiment imitation, where robots learn policies from demonstrations of these policies can be transferred to different robot embodiments, offers the potential for large-scale imitation learning that is both cost-effective and highly reusable.
   	      This paper presents <b>LEGATO</b>, a cross-embodiment imitation learning framework for visuomotor skills, facilitating the transfer of actions across various kinematic morphologies. 
              We introduce a wearable gripper that enables tasks to be defined within the same gripper's action and observation spaces across different robots.
	      Based on this wearable gripper, we train visuomotor policies through imitation learning, incorporating a motion-invariant transformation to compute the training loss. 
	      We then retarget gripper motions into high-DOF whole-body motions for deployment across diverse embodiments using inverse kinematics.
	     Our evaluation of simulations and real-robot experiments highlights the framework’s effectiveness in learning and transferring visuomotor skills across various robots.
	    </p>
          </td>
        </tr>
      </table>
    </p>
  </div>
</p>

<hr>
<h1 align="center">Cross-embodiment Learning Pipeline</h1>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <a href="./src/figure/overview.png"> 
          <img src="./src/figure/overview.png" style="width:598;">
        </a>
      </td>
    </tr> 
  </tbody> 
</table>

<table align=center width=800px>
  <tr>
    <td>
      <p align="justify" width="20%">
      	We introcuce a cross-embodiment imitation learning framework that enables human demonstrations via direct interaction or robot teleoperation. Our framework uses the LEGATO Gripper, a versatile wearable gripper, to maintain consistent physical interactions across different robots. During data collection, our gripper records the trajectories and grasping actions of the wearable gripper, as well as visual observations from its egocentric stereo camera. Policies trained on these gripper motions can be deployed on various robots equipped with the same gripper. Motion retargeting using IK optimization enables these trajectories to be executed on different robots without the need for training data specific to any particular robot.
      </p>
    </td>
  </tr>
</table>

<hr>
<h1 align="center">Wearable Gripper Design</h1>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr> 
      <td align="center" valign="middle">
        <video muted autoplay loop width="394">
          <source src="./src/video/mechanism.mp4"  type="video/mp4">
        </video>
      </td>
    </tr> 
  </tbody> 
</table>
<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <a href="./src/figure/hardware.png"> 
          <img src="./src/figure/hardware.png" style="width:394;">
        </a>
      </td>
      <td align="center" valign="middle">
        <video muted autoplay loop width="394">
          <source src="./src/video/assembly.mp4"  type="video/mp4">
        </video>
      </td>
    </tr> 
  </tbody> 
</table>

<table align=center width=800px>
  <tr>
    <td> 
      <p align="justify" width="20%">
        The LEGATO Gripper is designed for both collecting human demonstrations and robot deployment.
        Its design features a shared actuated gripper with handles adapted to each embodiment, ensuring robust human handling. 
        This ensures consistent grasping actions while minimizing the number of parts necessary for direct human use and deployment across various robots.
      </p>
    </td>
  </tr>
</table>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <video muted controls loop width="394">
          <source src="./src/video/usage_human.mp4"  type="video/mp4">
        </video>
      </td>
      <td align="center" valign="middle">
        <video muted controls loop width="394">
          <source src="./src/video/usage_robot.mp4"  type="video/mp4">
        </video>
      </td>
    </tr>
  </tbody>
</table>

<table align=center width=800px>
  <tr>
    <td> 
      <p align="justify" width="20%">
        The LEGATO Gripper allows a human demonstrator to directly perform tasks by carrying it, using a simple button interface. 
        It is also designed to be easily installed on various robot systems by simply holding it with their original grippers.
      </p>
    </td>
  </tr>
</table>

<hr>
<h1 align="center">Whole-body Motion retargeting</h1>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <video class="lazy-video" muted loop width="798">
          <source src="./src/video/whole-body_ik.mp4"  type="video/mp4">
        </video>
      </td>
    </tr> 
  </tbody> 
</table>

<table align=center width=800px>
  <tr>
    <td> 
      <p align="justify" width="20%">
        Motion retargeting through IK optimization adeptly navigates the kinematic differences and constraints across robot embodiments, exploiting kinematic redundancy without requiring additional robot-specific demonstrations for deployment.
      </p>
    </td>
  </tr>
</table>

<hr>
<h1 align="center">Real-robot Deployment</h1>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <video class="lazy-video" muted loop width="798">
          <source src="./src/video/real_panda.mp4"  type="video/mp4">
        </video>
      </td>
    </tr> 
  </tbody> 
</table>

<hr>
<h1 align="center">Simulation Evaluation</h1>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <a href="./src/figure/benchmark.png">
          <img src="./src/figure/benchmark.png" style="width:798;">
        </a>
      </td>
    </tr>
  </tbody>
</table>

<table align=center width=800px>
  <tr>
    <td>
      <p align="justify" width="20%">
    	On average, LEGATO  outperforms the other methods in cross-embodiment deployment by 28.9%, 10.5%, and 21.1%, compared to <a href="https://arxiv.org/abs/2108.03298">BC-RNN</a>, <a href="https://arxiv.org/abs/2303.04137v5">Diffusion Policy</a>, and the self-variant of LEGATO trained only on SE3 (LEGATO (SE3)), respectively.
    	Notably, unlike the baselines that only achieved high success rates on specific robot bodies, typically the <i>Abstract</i> embodiment used for training, LEGATO demonstrates consistent success across different embodiments.
      </p>
    </td>
  </tr>
</table>

<table border="0" cellspacing="10" cellpadding="0" align="center">
  <tbody>
    <tr>
      <td align="center" valign="middle">
        <video class="lazy-video" muted loop width="598">
          <source src="./src/video/sim_lid.mp4"  type="video/mp4">
        </video>
      </td>
    </tr>
    <tr>
      <td align="center" valign="middle">
        <video class="lazy-video" muted loop width="598">
          <source src="./src/video/sim_cup.mp4"  type="video/mp4">
        </video>
      </td>
    </tr>
    <tr>
      <td align="center" valign="middle">
        <video class="lazy-video" muted loop width="598">
          <source src="./src/video/sim_ladle.mp4"  type="video/mp4">
        </video>
      </td>
    </tr>
  </tbody>
</table>

<hr>
<center><h1>Citation</h1></center>

<table align=center width=800px>
  <tr>
    <td>
    <!-- <left> -->
    <pre><code style="display:block; overflow-x: auto">
      @misc{seo2024legato,
        title={LEGATO: Cross-Embodiment Visual Imitation Using a Grasping Tool},
        author={Seo, Mingyo and Park, H. Andy and Yuan, Shenli and Zhu, Yuke and
          and Sentis, Luis},
        year={2024}
        eprint={XXXX.XXXXXX},
        archivePrefix={arXiv},
        primaryClass={cs.RO}
      }
    </code></pre>
    <!-- </left> -->
    </td>
  </tr>
</table>

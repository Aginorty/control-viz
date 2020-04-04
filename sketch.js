
let canvas, canvasWidth = 700, canvasHeight = 400, lineToFollow;
var gui;

// line follower parameters
var kp = 0.0;
var kpMin = 0.0;
var kpMax = 0.5;
var kpStep = 0.001;

var ka = 0.00;
var kaMin = 0.0;
var kaMax = 1.5;
var kaStep = 0.01;

var myChoice = [1,2,3,4];

var defineLineButton;
var isFirstLinePointDefined = true;
var isSecondLinePointDefined = true;
var isInDefineLineState = false;
var renderFirstPointOnly =false;

var isInDefineInitialRobotPoseState = false;

var robot;
var maxForwardSpeed = 0.51;
var maxOmega = 0.1;

var endTargetError = 0.1;
var perpendiculartRobotToLineError = 0.1;
var angleRobotToLineErrorRad = 0.1;

var resetRobotPositionButton;
var isInResetRobotState = false;

function setup() {
  angleMode(RADIANS);
  canvas = createCanvas(canvasWidth, canvasHeight);
  gui = createGui('Controller parameters');
  gui.addGlobals('kp', 'ka');

  lineToFollow = new FollowLine();

  defineLineButton = createButton('Define line');
  defineLineButton.position(20, canvasHeight-30);
  defineLineButton.mousePressed(defineLine);

  resetRobotPositionButton = createButton('Reset robot pos');
  resetRobotPositionButton.position(20, canvasHeight - 60);
  resetRobotPositionButton.mousePressed(resetRobotPosition);

  canvas.mouseClicked(deFineLinePointsOrRobotInitialPose);

  robot = new Robot();
  kp = 0.1;
  ka = 0.05;

}

function draw() {
  background(51);
  lineToFollow.render(renderFirstPointOnly);
  robot.updateTargetErrors();
  robot.update();
  robot.render();
}

function Robot(x, y) {
  // x, y , omega
  this.acceleration = createVector(0.1, -0.1, 0.001);
  this.velocity = createVector(0.1, -0.1, 0.001);
  this.position = createVector(canvasWidth/3, canvasHeight*(2/3), -HALF_PI);
  this.r = 3.0;
  this.maxspeed = 3;    // Maximum speed
  this.maxforce = 0.05; // Maximum steering force
}

Robot.prototype.render = function(){
  let c = color(245, 172, 27);
  fill(c);
  var fisrtPoint
  push();
  translate(this.position.x, this.position.y);
  rotate(this.position.z);
  //triangle(0,0,5,10,-5,10);
  triangle(0, 0, -10, -5, -10, 5);
  pop();
}

Robot.prototype.update = function(){

  var projectionAngle = this.position.z + this.velocity.z/2;
  this.position.x += this.velocity.y * Math.cos(projectionAngle);
  this.position.y += this.velocity.y * Math.sin(projectionAngle);
  this.position.z += this.velocity.z;
  this.position.z = wrapRad(this.position.z);

  this.velocity.y = 0.005*endTargetError;
  if(Math.abs(angleRobotToLineErrorRad) < Number.EPSILON){
    angleRobotToLineErrorRad = 0.00001;
  }
  var perpendicularControlContribution = kp*perpendiculartRobotToLineError * Math.sin(angleRobotToLineErrorRad)/angleRobotToLineErrorRad;
  var angularControlContribution = ka*angleRobotToLineErrorRad;
  this.velocity.z = -perpendicularControlContribution + angularControlContribution;
  //console.log(kp*endTargetError);
  // if(this.velocity.y > maxForwardSpeed){
  //   this.velocity.y = maxForwardSpeed;
  // }
  // if(this.velocity.y < -maxForwardSpeed){
  //   this.velocity.y = -maxForwardSpeed;
  // }
  //
  //
  if(this.velocity.z > maxOmega){
    this.velocity.z = maxOmega;
  }
  if(this.velocity.z < -maxOmega){
    this.velocity.z = -maxOmega;
  }

}

Robot.prototype.updateTargetErrors = function(){
  var robotToLineSecondPointVector = createVector(this.position.x -lineToFollow.secondPoint.x, this.position.y - lineToFollow.secondPoint.y)
  // find the sign of the error
  var signOfError = 1;
  if(lineToFollow.firstPoint.y <= lineToFollow.secondPoint.y ){
    if(this.position.y > lineToFollow.secondPoint.y){
      signOfError = -1;
    }
  }

  if(lineToFollow.firstPoint.y > lineToFollow.secondPoint.y ){
    if(this.position.y < lineToFollow.secondPoint.y){
      signOfError = -1;
    }
  }


  endTargetError = robotToLineSecondPointVector.mag() * signOfError;
  //console.log(endTargetError);
  var lineDirectVector = createVector(-lineToFollow.headingVector.y, lineToFollow.headingVector.x);
  perpendiculartRobotToLineError = robotToLineSecondPointVector.dot(lineDirectVector)/lineDirectVector.mag() / 100;
  //console.log(perpendiculartRobotToLineError);

  angleRobotToLineErrorRad = smallestSignedDiffRad(this.position.z , lineToFollow.heading);
  //console.log(angleRobotToLineErrorRad);
}

function FollowLine() {
  this.firstPoint = createVector(canvasWidth/2, 20);
  this.secondPoint = createVector(canvasWidth/2, canvasHeight-20);

  //var headingVector = createVector(this.firstPoint.x-this.secondPoint.x, this.firstPoint.y - this.secondPoint.y);
  this.headingVector = createVector(this.secondPoint.x-this.firstPoint.x, this.secondPoint.y - this.firstPoint.y);
  this.heading = this.headingVector.heading();
}

FollowLine.prototype.render = function(renderFirstOnly){
  strokeWeight(2);
  push();

  let c = color(245, 172, 27);
  fill(c);
  if(!renderFirstOnly){
    stroke(c);
    line(this.firstPoint.x, this.firstPoint.y, this.secondPoint.x, this.secondPoint.y);
    textSize(15);
    strokeWeight(0.5);
    fill(245, 172, 27);
    text('2', this.secondPoint.x + 10, this.secondPoint.y + 10);
    stroke(0);
    circle(this.secondPoint.x, this.secondPoint.y, 10);
  }

  textSize(15);
  strokeWeight(0.5);
  fill(245, 172, 27);
  text('1', this.firstPoint.x + 10, this.firstPoint.y + 10);
  stroke(0);
  circle(this.firstPoint.x, this.firstPoint.y, 10);
  pop();
}

function defineLine(){
  isInDefineLineState = true;
  isFirstLinePointDefined = false;
  isSecondLinePointDefined = false;
}

function resetRobotPosition(){
  isInResetRobotState = true;
  console.log("in reset robot state");
}

function deFineLinePointsOrRobotInitialPose(){
  if(isInDefineLineState){
    if(!isFirstLinePointDefined){
      lineToFollow.firstPoint = createVector(mouseX, mouseY);
      isFirstLinePointDefined = true;
      renderFirstPointOnly = true;
    }else if(!isSecondLinePointDefined){
      lineToFollow.secondPoint = createVector(mouseX, mouseY);
      isInDefineLineState = false;
      renderFirstPointOnly = false;
      lineToFollow.headingVector = createVector(lineToFollow.secondPoint.x-lineToFollow.firstPoint.x, lineToFollow.secondPoint.y - lineToFollow.firstPoint.y);
      lineToFollow.heading = lineToFollow.headingVector.heading();
    }
  }else if(isInResetRobotState){
    robot.position.x = mouseX;
    robot.position.y = mouseY;
    isInResetRobotState = false;

  }
}

function wrapRad(angle){
  var tmp = angle % (2 * Math.PI);
  tmp = (tmp + 2 * Math.PI) % (2 * Math.PI);
  return (tmp > Math.PI) ? (tmp - 2 * Math.PI) : tmp;
}

function smallestSignedDiffRad(start, target) {
    start = wrapRad(start);
    target = wrapRad(target);

    var diff = target - start;
    diff += (diff >= Math.PI) ? -2 * Math.PI : (diff < -Math.PI) ? 2 * Math.PI : 0;

    return diff;
  }

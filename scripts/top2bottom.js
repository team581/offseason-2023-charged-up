#!/usr/bin/env node

// @ts-check

const fs = require("fs/promises");
const nodePath = require("path");

function serialize(path) {
  return JSON.stringify(path, null, 2);
}

function translate(waypoint) {
  const { y } = waypoint.anchorPoint;

  waypoint.anchorPoint.y = 2.7432 - (y - 2.7432);
  waypoint.holonomicAngle = -waypoint.holonomicAngle;
  waypoint.holonomicAngle %= 360;

  if (waypoint.prevControl) {
    waypoint.prevControl.y = 2.7432 - (waypoint.prevControl.y - 2.7432);
  }

  if (waypoint.nextControl) {
    waypoint.nextControl.y = 2.7432 - (waypoint.nextControl.y - 2.7432);
  }
}

async function main () {

const pathDir = nodePath.join(
  __dirname,
  "..",
  "src",
  "main",
  "deploy",
  "pathplanner"
);

const [, , ...rawFilenames] = process.argv;

let filenames = rawFilenames;

// Default to all blue paths
if (filenames.length === 0) {
  const dir = await fs.readdir(pathDir);

  filenames = dir
    .filter(
      (filename) => filename.startsWith("Blue") && filename.endsWith(".path")
    )
    .map((absolutePath) => nodePath.basename(absolutePath));
}

for (const filename of filenames) {
  const pathBuf = await fs.readFile(nodePath.join(pathDir, filename));
  const path = JSON.parse(pathBuf.toString());

  for (const waypoint of path.waypoints) {
    translate(waypoint);
  }

  const outputPath = nodePath.join(
    pathDir,
    `Bottom_${filename}`
  );
  await fs.writeFile(outputPath, serialize(path));
  console.log(outputPath);
}
}

main();

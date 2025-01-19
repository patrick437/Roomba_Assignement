# EE5108-Digital-Twins-for-Robotics
Github repository for Digital Twins Module


# Git Installation and Basic Usage Guide

This guide provides step-by-step instructions for installing Git, setting up a workspace directory, and cloning a repository. This is essential for managing and collaborating on software projects using version control.

## Prerequisites

Before you begin, ensure you have the following:

- A working internet connection.

## Installing Git

### On Ubuntu/Debian

1. **Update Package Index**

   Open a terminal and run the following command to update your package index:

   ```bash
   sudo apt-get update
   ```

2. **Install Git**

   Install Git using the package manager:

   ```bash
   sudo apt-get install git
   ```

3. **Verify Installation**

   Check if Git was installed correctly by checking its version:

   ```bash
   git --version
   ```

### On Windows

1. **Download Git**

   Go to the [Git for Windows](https://gitforwindows.org/) website and download the installer.

2. **Run the Installer**

   Run the downloaded executable and follow the installation instructions. During the installation, you can choose the default options unless specific changes are needed.

3. **Verify Installation**

   Open Command Prompt or Git Bash and type:

   ```bash
   git --version
   ```

### On macOS

1. **Install via Homebrew**

   If you have Homebrew installed, you can install Git with:

   ```bash
   brew install git
   ```

2. **Verify Installation**

   Verify the installation with:

   ```bash
   git --version
   ```

## Setting Up a Workspace

1. **Create a New Directory**

   Choose a location on your computer where you would like to keep your projects. Open a terminal or command prompt and navigate to that location. Then, create a new directory:

   ```bash
   mkdir ee5108_workspace
   ```

2. **Navigate to Your Workspace**

   Move into your newly created directory:

   ```bash
   cd ee5108_workspace
   ```

## Cloning a Git Repository

1. **Obtain Repository URL**

   Navigate to your desired repository on platforms like GitHub, GitLab, or Bitbucket. Click the "Code" button and copy the repository's URL.

2. **Clone the Repository**

   In the terminal or command prompt, use the following command to clone the repository into your workspace directory:

   ```bash
   git clone https://github.com/CARinternal/EE5108-Digital-Twins-for-Robotics.git
   ```

3. **Access Your Cloned Repository**

   Once cloned, navigate into the repository directory:

   ```bash
   cd EE5108-Digital-Twins-for-Robotics
   ```




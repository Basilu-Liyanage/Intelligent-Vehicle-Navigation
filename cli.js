const axios = require('axios');
const { Command } = require('commander');
const program = new Command();

// GitHub Personal Access Token (replace with your actual token)
const GITHUB_TOKEN = 'YOUR_GITHUB_TOKEN';

// Set up the base GitHub API URL
const BASE_URL = 'https://api.github.com';

// Set up Axios instance
const axiosInstance = axios.create({
  baseURL: BASE_URL,
  headers: {
    'Authorization': `token ${GITHUB_TOKEN}`,
  },
});

// Function to get user information from GitHub
async function getUserInfo(username) {
  try {
    const response = await axiosInstance.get(`/users/${username}`);
    console.log(`User Info for ${username}:`);
    console.log(`Name: ${response.data.name}`);
    console.log(`Bio: ${response.data.bio}`);
    console.log(`Public Repos: ${response.data.public_repos}`);
  } catch (error) {
    console.error(`Error fetching user data: ${error.response.data.message}`);
  }
}

// Set up your commands
program
  .command('user <username>')
  .description('Get information about a GitHub user')
  .action(async (username) => {
    await getUserInfo(username);
  });

// Parse the command-line arguments
program.parse(process.argv);

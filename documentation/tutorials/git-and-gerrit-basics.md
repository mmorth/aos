# Git and Gerrit Basics

Git and Gerrit are both essential tools which we use to program the robot, this document goes over the basics of both tools and some helpful information on their use.

* [What is Git?](#what-is-git)
    * [Git basics](#git-basics)
        * [Commits](#commits)
        * [Branches](#branches)
        * [Helpful commands](#helpful-commands)
* [What is Gerrit?](#what-is-gerrit)
    * [Gerrit basics](#gerrit-basics)

## What is Git?

Term Definition:
- Repository: A project or collection of projects managed using git.
For example our code lives in a repository called `RealtimeRoboticsGroup/aos`.
- Commit: A collection of code changes
- Branch: A collection of commits

Git is a tool we use which manages changes to our code. Git allows you to track changes, and combine them back into the version of the code that the robot runs off of.

One of the ways git tracks changes to our code is **commits**, this may deviate slightly from what you're used to if you've used github before.
A commit contains the code of one change, as well a title and description of the change.

Another way to manage your changes locally are **branches**.
Branches contain a stack of commits, and branches are used to combine commits back into the code which runs on the robot.
This is called the **main branch**, and is the stack of commits which runs on the robot.

Once your change has finished you need to **push** it.
Pushing your commit places it online so that it can be checked, and then put into the main branch.

### Git basics

When you make a new file in a git repository, its considered *untracked*.
You can see this by doing `git status`, this shows you the status of all the files in your change, before they're committed.
To track it you have to do `git add <FILENAME>`, now if you do git status you'll see its green and tracked.
If you're editing an already tracked file it'll show up as tracked, but won't be added until you do git add

#### Commits

Once you have a change finished, and all the files added, you can do `git commit -s`.
This then brings up your text editor and lets you edit the commit message and description.

The commits are formatted like this:

```
Title

Description

...
```

Heres a good blog post about formatting commit messages: [link](https://cbea.ms/git-commit/)

Once you have committed it you can push the code to our repository using `git push origin HEAD:refs/for/main`

To break this command down you are pushing to origin, which links to the online repository, `refs/for/main` tells it to make a gerrit commit review for the main branch.


#### Branches

Whenever you need to manage multiple commits you'll be doing them accross multiple branches.
Here are the basics of branch management:

```bash
git branch <NAME> # Creates a new branch with the given name
git branch -D <NAME> # Deletes the branch with the given name
git switch <NAME> # Switches to the branch with the given name
```

If you need to switch to a different branch, but the change you have on your current branch hasn't been committed.
You can do `git stash`, which will store your uncommitted changes until you do `git stash pop`.

#### Helpful commands

```bash
# This updates your local code to match the latest version
git switch main
git pull

# Rebases your current commit onto the latest main branch
git pull origin main --rebase
```

## What is Gerrit?

Gerrit is a similar tool to github, except that instead of making changes and doing reviews on branches, you do them on commmits.

### Gerrit basics

If you have a commit on gerrit that you want to edit you can press d when on the commit or click download. Then select checkout.

Paste the command into the terminal and do `git switch -c <NAME>`. The name can be anything, as its only used to manage local development.

Once you've made your changes to that commit, you can do `git commit --amend -s`, which will add your new changes to the commit.

Occasionally you'll need to edit someone elses commit.
When you are pushing you'll get an error which states that you cannot override another author's commit.
To get around this you can do `git commit --amend --reset-author -s`

## Setup and Configuration

### GitHub

To contribute back to AOS you will need both github and gerrit setup with matching ssh keys.
`ssh-keygen -t ed25519 -C "your-email@example.com"`

You will need to add the following line to your `~/.bashrc` file
```
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519<_if not default>
```
[Add key] (link to directions) to github.
In `~/.ssh/config` tell git to use this key
```
Host github-aos
  HostName github.com
  User git
  IdentityFile ~/.ssh/id_ed25519<_if not default>
  IdentitiesOnly yes
```
Check your repo is setup correctly.
`git remote set-url origin git@github-aos:RealtimeRoboticsGroup/aos.git`

Test it `ssh -T git@github-aos`

### Gerrit

In `~/.ssh/config` tell gerrit to use this key
```
Host gerrit
  HostName realtimeroboticsgroup.org
  Port 29418
  User <your_gerrit_username>
  IdentityFile ~/.ssh/id_ed25519<_if not default>
  IdentitiesOnly yes

```
Reach out or file a ticket to get yourself added to the Verified Users group so you can trigger CI and push to `unreviewed/${username}` branches.
Add the gerrit romte ssh to git `git remote add gerrit "ssh://<gerrit user>@realtimeroboticsgroup.org:29418/RealtimeRoboticsGroup/aos"`
Then add some nice hooks to automatically add Change-Id to each commit:
```
mkdir -p `git rev-parse --git-dir`/hooks/ && curl -Lo `git rev-parse --git-dir`/hooks/commit-msg https://realtimeroboticsgroup.org/gerrit/tools/hooks/commit-msg && chmod +x `git rev-parse --git-dir`/hooks/commit-msg
```
and finally configure git to push to getrrit:
`git remote set-url gerrit ssh://gerrit/RealtimeRoboticsGroup/aos`
Test this with `ssh -p 29418 gerrit`

Finally you should be able to push a branch to unrviewed to test that you can:
Create a new branch with some trivial change named `unreviewed/<gerrit user>/<branch name>`
Make sure this change is signed off by using the git `-s` option during commit or running `git commit --amend -s`, more info can found [here.](https://github.com/RealtimeRoboticsGroup/aos?tab=readme-ov-file#contributing)
Now lets push our change to gerrit, but not make a PR to review.
`git push gerrit HEAD:refs/heads/unreviewed/<gerrit user>/<branch name>`
There is possibilty this will fail, but everything will be working.
You might just lack create premissions.
If you want to make a real PR try `git push gerrit HEAD:refs/for/main`
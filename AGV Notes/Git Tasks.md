- ## Merging
  1. Normal merging
    * We checkout to the branch and `git merge <branch-to-be-merged>` so that the branch merges to the currently checked out branch.
  
  3. Rebasing
    * Rebasing essentially takes a set of commits, "copies" them, and plops them down somewhere else.
  
    * While this sounds confusing, the advantage of rebasing is that it can be used to make a nice linear sequence of commits. The commit log / history of the repository will be a lot cleaner if only rebasing is allowed.
  
    * `git rebase main` - This rebases the current branch to the main branch 
  
    * Next we rebase the the main branch to the rebased branch by `git rebase <rebased-branch>`
---
- ## Moving around in Git
  1. **HEAD**
	    * HEAD is the symbolic name for the currently checked out commit -- it's essentially what commit you're working on top of.

	    * HEAD always points to the most recent commit which is reflected in the working tree. Most git commands which make changes to the working tree will start by changing HEAD.
	  
	    * Normally HEAD points to a branch name (like bugFix). When you commit, the status of bugFix is altered and this change is visible through HEAD.
	  
	    * **Detaching HEAD :** Detaching HEAD just means attaching it to a commit instead of a branch.
	
  2. **Relative Refs**
	    *  `git checkout <commmit-code>`
	    * `git checkout HEAD^` : HEAD points to the previous commit
	    * `git checkout HEAD~3`: HEAD goes to the commit three times back
	    * `git main^` : HEAD goes to the parent commit of latest main commit
  
  3. **Branch forcing**
	    * You're an expert on relative refs now, so let's actually use them for something.
	  
	    * You can directly reassign a branch to a commit with the -f option. So something like: `git branch -f main HEAD~3`. This makes the main branch point to the commit 3 times back.
	---
- ## Reversing git changes
  
  1. **Git Reset**
	    * `git reset` : Reverses changes by moving a branch reference backwards in time to an older commit. In this sense you can think of it as "rewriting history;" git reset will move a branch backwards as if the commit had never been made in the first place.
  
  2. **Git Revert**
	    * `git revert` : While resetting works great for local branches on your own machine, its method of "rewriting history" doesn't work for remote branches that others are using.
	  
	    * In order to reverse changes and share those reversed changes with others, we need to use git revert.

  3. **Git Squash**
		* Combine multiple commits into one single commit.
		*  `git rebase -i HEAD~3` : Allows us to *edit* the last 3 commits. We can squash them / change commit messages and etc. 
		* 
	---
 - ## Remote 
	  1. **Remote Branches**
	      * The first thing you may have noticed is that a new branch appeared in our local repository called `o/main`. This type of branch is called a remote branch; remote branches have special properties because they serve a unique purpose.
	  
	      * Remote branches reflect the state of remote repositories (since you last talked to those remote repositories). They help you understand the difference between your local work and what work is public -- a critical step to take before sharing your work with others.
	  
	      * Remote branches have the special property that when you check them out, you are put into detached `HEAD` mode. Git does this on purpose because you can't work on these branches directly; you have to work elsewhere and then share your work with the remote (after which your remote branches will be updated).
	  
	      * To be clear: Remote branches are on your local repository, not on the remote repository.
	
	  2. **Fetching**
	     * Downloads the commits that the remote has but are missing from our local repository, and...
	      
	      * updates where our remote branches point (for instance, `origin/main`)
	  
	      * `git fetch`, however, does not change anything about your local state. It will not update your main branch or change anything about how your file system looks right now.
	  
	      * It may download all the necessary data to do that, but it does not actually change any of your local files.
	  
	      * So at the end of the day, you can think of running `git fetch` as a download step.
	  
	      * It gets the commits across all the branches
	
	  3. Pull
	  
		   * In fact, the workflow of fetching remote changes and then merging them is so common that git actually provides a command that does both at once! That command is `git pull`.
	
		   * `git pull` is essentially shorthand for a `git fetch` followed by a merge of whatever branch was just fetched.
	 ---
## Git Editor
* To start editing press the `A` key. The text `INSERT` will appear at the bottom.
* To exit `INSERT` mode, Press the `Esc` key. The `INSERT` at the bottom will disappear.
* To save your changes and exit the document, type `:wq!` and press `Enter` key.
* To exit the document without saving, type `:q!` and press `Enter` key.

	
Making Changes
===

---

## Making changes to a Robot repository

1. Create a branch and make your changes in that branch.

   git checkout -b my_awesome_change

1. Test the change. Ideally your change has some kind of unit test.

   You can run tests from Visual Code or from gradle.

       ./gradlew check

   Check will run the tests as well as checkstyle and pmd analysis on it.
   This will take some time. If you just want to run the tests you can

       ./gradlew test

   And if you want to run just the new test you wrote:

       ./gradlew test <class name of test fixture>

1. Test the change in the simulator and/or on a robot if applicable.

1. Review the change.
   * In Visual Code
     * Select the file from the list of "Changes" in the
       Visual Code **Source Control** view.
     * It should display a diff with the old version on the left and new
       version on the right
     * Deletes from the old will be in red. Inserts to the new will be green.
   
1. Commit and push the branch to github.
   * Use visual code or `git push origin <branch>`

1. Review the change in github and send a Pull Request.

1. Wait for someone to review your change. Ping them if you are in a hurry.

---

## Contributing changes to CougarLib

### Overview

There are several reasons to make changes in CougarLib. Among them:

  * You want to fix a bug in an existing component.
  * You want to add a new component.
  * You want to refactor a piece of your Robot into CougarLib so
    that it is shareable among other Robots.

If you are just fixing a bug or adding a new component, it may be
simpler to clone the CougarLib repository and make the change there
as any other git commit / pull request. But you do not need to.

*  Be careful when making changes to CougarLib as they may break other
robots when they update their libraries. You should take care to write
tests that cover any changes you make.

* If you are making a breaking change (e.g. changing an interface) that other
people are relying on, then you should try to deprecate the old interface
and add a new one. That means mark the old interface as @deprecated but
leave it (and its tests) around so other people can still use it.
Introduce your new interface along side it and give people a chance to
migrate to it. Then remove the old deprecated one.

### Instructions
1. From your Robot's workspace, create a branch in CougarLib.
   * If you are also changing your robot then you will need a second branch
     from your workspace root (because it is your Robot repository).

         cd CougarLib
         git checkout -b my_awseome_change
         ...

2. Test your change

   * You can test the CougarLib change in your local workspace before 
     you commit it or push it up for a PR.

3. Follow github pull-request workflow
   * Each of these changes will follow the normal github pull request
     workflow outlined above, but they are independent of one another.

   * Submit the CougarLib one first because your Robot change needs it
     and it will not yet be available to other workspaces until you submit it.

---

## Updating external submodules (e.g. CougarLib)

### Overview

The submodule dependencies in the Robot (e.g. CougarLib) are to a particular
commit. When the CougarLib changes, you will not see those changes. This is
good because it provides stability and changes in CougarLib will not break
your robot.

However that also means you will not benefit from those changes. Ideally you
should try to stay current with CougarLib so you will need to update your
repository.

### Instructions

1. Update the submodule

       git checkout -b "update_cougarlib"
       git submodule update --remote CougarLib

1. Test your robot with the changes

   * If the update broke your robot then debug the problem to determine
     if the bug is in your robot or in the change.

   * If the bug was in your robot, fix it.

   * If the bug was in the external component then take whatever appropriate
     action is to fix or report the issue.

   * If you want to give up on making the change, switch out of the branch
     (e.g. back to main) and the earlier version associated with that branch
     will automatically restore.
         * Delete the branch you created to attempt the update.

1. Submit the changes
   * You will need to eventually submit that change so that the
     repository knows to advance its use of CougarLib to the commit
     you updated to.

---

## Changing the TemplateRobotLibrary itself

### Overview
Changes to the template library are like any other, however should be done
in an explicit clone of the TemplateRobotRepository, not from a derived
custom Robot repository.

### Instructions
1. Clone the templateRobotLibrary

   git clone https://github.com/FRC-Team-1403/TemplateRobotRepository.git

1. Make the change following the standard change process.

   * To test your change:

     1. Create a new robot using a local repository

            mkdir MyTestRobot.origin
            cd MyTestRobot.origin
            git init
            cd ..

            git Clone MyTestRobot.origin MyTestRobot
            cd MyTestRobot
            git remote add template ../TemplateRobotRepository
      
        At this point your MyTestRobot repository has 2 remotes
          * origin is ../MyTestRobot.origin
          * template  is ../TemplateRobotRepository (with your change)

            git fetch template
            git checkout <branch in your local template>
            ./setup_robot.sh "My Test Robot"

        This should create the robot and push changes to ../MyTestRobot.origin

     1. Have a look around to make sure your change did what it was supposed to.

     1. Delete those scratch repositories.

            # Go back to parent directory of your cloned repositories
            # so you can remove them
            cd .. 

            rm -rf MyTestRobot.origin
            rm -rf MyTestRobot


## Syncing to changes in the Template Repository

### Overview

The repository you created from the template will not automatically
be updated with changes made to the template. In fact git has no
replationship between the original template and your repository.

If you want to benefit from new features and fixes added into the
template repository since you copied it (or last synced) then you
will need to do this manuallly. These instructions show how to
cherry pick changes, which should be relatively easy.

### Instructions

1. Add the template repository as a remote

   `git remote add template https://github.com/FRC-Team-1403/TemplateRobotRepository.git`

1. Checkout the head of the template repository

   `git checkout template/main --`

   The trailing `--` tells git that `template/main` was the commit, not a path.

1. Create a new branch in your repository to cherry pick into.

   `git checkout main`

   `git checkout -b cherry_picks`

1. List the new commits since some reference commit.

   We want to know what changed since we last synced.
   We can look at the commit history for a commit within
   the last sync point. For example one that existed when
   we cloned the repository or we last synced to. We should
   write this down (e.g. in a file called "last_template_sync")
   for next time.

   This command can also use dates (e.g. 09/20/2022) or relative time
   (e.g. "--since yesterday", "--since last-month", "--since july").

   Lets assume that commit is $PREV_COMMIT.

   `git log --reverse --oneline template/main ${PREV_COMMIT}..HEAD`

   This will list all the commit ids from that last sync point
   to the current HEAD of the branch we checked out along with the
   first-line summary.  We reversed the order so the oldest ones are
   first, which is the order we'd want to cherry pick them.

1. Cherry pick the commits

   Where OLDEST_COMMIT is the first in the range
   and NEWEST_COMMIT is the last in the range (or empty if just the one commit)

   If you want the whole range, then you can cherry pick them all at once

   `git cherry-pick -x ${OLDEST_COMMIT} ${NEWEST_COMMIT}`

   You may encounter conflicts. If so then resolve the conflict (like you would
   a merge conflict) and then

   `git cherry-pick --continue`

   or

   `git cherry-pick --abort`

   to abort and either start over or skip it.

1. Review and test the branch

   At this point your branch is like any other change.
   You should review and test it to be sure it is as expected.
   Especially if you encountered conflicts.

   Once tested you can upload to github and send a PR to merge into the main branch.



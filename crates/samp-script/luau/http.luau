--!strict
-- Async HTTP over the host's `httpFetch`/`httpPoll` (reqwest on the tokio runtime), shaped like the
-- stock MoonLoader `asyncHttpRequest(method, url, args, resolve, reject)` so ported scripts (captcha
-- solver, Telegram reporting) change minimally. `args` = { headers = {name=value,..}, data = "body",
-- timeout = ms }. The request runs off-thread; this polls it from the newTask/wait scheduler and
-- fires `resolve(bodyText)` / `reject(errText)` on the single-threaded VM.

type Args = { headers: { [string]: string }?, data: string?, timeout: number? }

local function asyncHttpRequest(
    method: string,
    url: string,
    args: Args?,
    resolve: ((string) -> ())?,
    reject: ((string) -> ())?
)
    local onOk = resolve or function(_: string) end
    local onErr = reject or function(_: string) end
    local id: number = httpFetch(method, url, args)
    newTask(function()
        while true do
            wait(50)
            local status: string, text: string = httpPoll(id)
            if status == "ok" then
                onOk(text)
                return
            elseif status == "err" then
                onErr(text)
                return
            end
        end
    end)
end

return {
    asyncHttpRequest = asyncHttpRequest,
}
